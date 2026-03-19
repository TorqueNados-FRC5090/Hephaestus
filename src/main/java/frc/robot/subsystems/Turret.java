package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

// CTRE Phoenix 6 Imports
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// WPILib Imports
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    // --- Hardware & Control ---
    private final TalonFXS m_turretMotor;
    private final MotionMagicVoltage m_motionMagic;

    // --- External Dependencies ---
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotVelocitySupplier; // SOTM: Added Velocity
    private final AprilTagFieldLayout m_atLayout;

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -4; // Backwards
    private final double kTurretOffsetYInches = -5;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches), 
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // --- HUB OFFSET CORRECTION ---
    // Tune these to push the target coordinate from the Tag face to the True Center of the Hub.
    private final double kHubCenterOffsetXInches = 3.0; // Start with 12 inches, tune until centered
    private final double kHubCenterOffsetYInches = 0.0;  // Tune if tags are physically off-center left/right

    // --- Mechanical Constants ---
    private final double kTurretRingTeeth = 200.0; 
    private final double kEncoderGearTeeth = 16.0; 
    private final double kTurretGearRatio = kTurretRingTeeth / kEncoderGearTeeth; 
    private final double kMaxTurretRotations = 0.48; 

    // --- LIVE STATE VARIABLES ---
    public double m_distanceToHubMeters = 0.0;
    public double m_distanceToPassTargetMeters = 0.0;
    public double m_virtualDistanceToHubMeters = 0.0; // SOTM: Virtual targeting distance
    private double m_targetMotorRotations = 0.0;
    private boolean m_canSeeTarget = false;

    public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier, AprilTagFieldLayout atLayout) {
        this.m_robotPoseSupplier = poseSupplier;
        this.m_robotVelocitySupplier = velocitySupplier; // SOTM: Added Velocity
        this.m_atLayout = atLayout;

        m_turretMotor = new TalonFXS(16, "Upper"); 
        
        m_motionMagic = new MotionMagicVoltage(0);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.Slot0.kP = 8; 
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        
        config.MotionMagic.MotionMagicCruiseVelocity = 600.0; 
        config.MotionMagic.MotionMagicAcceleration = 60.0;   
        config.MotionMagic.MotionMagicJerk = 1600.0;          

        m_turretMotor.getConfigurator().apply(config);
        m_turretMotor.setPosition(0);
    }

    public void zeroTurret() {
        m_turretMotor.setPosition(0.0);
    }

    public double getDistanceToHubMeters() {
        return m_distanceToHubMeters;
    }

    /** * SOTM: This is the getter that the Shooter and Hood will use to 
     * dynamically adjust their equations based on whether we are passing or shooting.
     */
    public double getShootingDistance() {
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            return m_distanceToPassTargetMeters;
        }
        return m_virtualDistanceToHubMeters; 
    }

    /** Commands the motor to snap to the target. */
    public void alignToHub() {
        if (!m_canSeeTarget) return;
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    // Command that returns the Turret to the zero point when not in use
    public void goToZero() {
        m_turretMotor.setControl(m_motionMagic.withPosition(0));
    }

    /** Checks the turret if it is at the setpoint */
    public boolean isTurretReady(){
        if (m_targetMotorRotations == 0.0) {
            return false;
        }
        double currentpos = m_turretMotor.getPosition().refresh().getValueAsDouble();
        return Math.abs(currentpos - m_targetMotorRotations) <= 0.2;
    }

    /**
     * Call this when pressing the shoot button! 
     * It intelligently decides to pass if past midfield, or shoot at the hub if close.
     */
    public void passOrShoot() {
        Pose2d robotPose = m_robotPoseSupplier.get();
        boolean isRed = isRedAlliance();
        
        // FRC field dimensions
        double fieldLength = 16.54;
        double fieldWidth = 8.21; 
        double fieldMidpointX = fieldLength / 2.0; 
        double hubCenterY = fieldWidth / 2.0; // ~4.105 meters
        
        // Check if we are past the midline into the opponent's zone
        boolean inOpponentOrMidZone = isRed ? (robotPose.getX() <= fieldMidpointX) : (robotPose.getX() >= fieldMidpointX);

        if (inOpponentOrMidZone) {
            // ==========================================
            // --- PASSING & HUB AVOIDANCE LOGIC ---
            // ==========================================
            double passTargetX = isRed ? fieldLength : 0.0;
            double passTargetY = robotPose.getY();
            
            double dangerZoneClearanceMeters = 1.5; 

            // Shift target to the side if lined up with the hub net
            if (Math.abs(robotPose.getY() - hubCenterY) < dangerZoneClearanceMeters) {
                if (robotPose.getY() >= hubCenterY) {
                    passTargetY = hubCenterY + dangerZoneClearanceMeters;
                } else {
                    passTargetY = hubCenterY - dangerZoneClearanceMeters;
                }
            }
            
            passTargetY = MathUtil.clamp(passTargetY, 0.5, fieldWidth - 0.5);
            Translation2d passTarget = new Translation2d(passTargetX, passTargetY);

            Translation2d globalTurretPos = robotPose.getTranslation()
                .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

            Translation2d turretToPassTarget = passTarget.minus(globalTurretPos);
            
            m_distanceToPassTargetMeters = turretToPassTarget.getNorm();
            SmartDashboard.putNumber("Turret/Pass_Distance_Meters", m_distanceToPassTargetMeters);

            Rotation2d turretSetpoint = turretToPassTarget.getAngle()
                .minus(robotPose.getRotation())
                .minus(Rotation2d.fromDegrees(180));

            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_turretMotor.setControl(m_motionMagic.withPosition(desiredTurretRotations * kTurretGearRatio));
            
            SmartDashboard.putString("Turret/Mode", "PASSING");
            SmartDashboard.putNumber("Turret/Pass_Target_Y", passTargetY);
        } else {
            // Normal shooting in our zone
            SmartDashboard.putString("Turret/Mode", "SHOOTING");
            alignToHub();
        }
    }

    private boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // 1. --- LIVE MOTOR DATA ---
        double currentMotorRotations = m_turretMotor.getPosition().refresh().getValueAsDouble();
        double currentTurretRotations = currentMotorRotations / kTurretGearRatio;
        SmartDashboard.putNumber("Turret/Current_Motor_Rots", currentMotorRotations);
        SmartDashboard.putNumber("Turret/Current_Turret_Rots", currentTurretRotations);

        // 2. --- APRILTAG ACQUISITION ---
        boolean isRed = isRedAlliance();
        int targetTag = isRed ? 10 : 26; 
        Optional<Pose3d> tagPoseOpt = m_atLayout.getTagPose(targetTag);
        
        if (tagPoseOpt.isEmpty()) {
            m_canSeeTarget = false;
            m_distanceToHubMeters = 0.0; 
            m_virtualDistanceToHubMeters = 0.0;
            SmartDashboard.putString("Turret/STATUS", "ERROR: Tag " + targetTag + " not found!");
            return; 
        } 
        
        m_canSeeTarget = true;
        SmartDashboard.putString("Turret/STATUS", "Tracking Tag " + targetTag);
        
        Pose2d hubPose = tagPoseOpt.get().toPose2d();
        Translation2d hubTranslation;
        
        // Apply physical hub offsets
        Translation2d hubCorrectionOffset = new Translation2d(
            Units.inchesToMeters(kHubCenterOffsetXInches), 
            Units.inchesToMeters(kHubCenterOffsetYInches)
        );

        if (!isRed) {
            hubTranslation = hubPose.getTranslation().minus(hubCorrectionOffset);
        } else {
            hubTranslation = hubPose.getTranslation().plus(hubCorrectionOffset);
        }
            
        Pose2d robotPose = m_robotPoseSupplier.get(); 
        Translation2d globalTurretPos = robotPose.getTranslation()
            .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

        // Calculate physical distance
        Translation2d turretToHub = hubTranslation.minus(globalTurretPos);
        m_distanceToHubMeters = turretToHub.getNorm();
        SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);

        // ==========================================================
        // 3. --- SHOOT ON THE MOVE (VIRTUAL TARGET) MATH ---
        // ==========================================================
        
        // A. Get current robot velocity
        var speeds = m_robotVelocitySupplier.get();
        double robotVelX = speeds.vxMetersPerSecond;
        double robotVelY = speeds.vyMetersPerSecond;

        // B. Estimate Shot Speed (Meters per second) - TUNE THIS ON THE REAL ROBOT!
        double kEstimatedShotSpeedMPS = 12.0; 

        // C. Calculate Time of Flight (t = d/v)
        double timeOfFlight = m_distanceToHubMeters / kEstimatedShotSpeedMPS;

        // D. Calculate how much the ball will drift due to robot movement
        Translation2d inheritedVelocityOffset = new Translation2d(
            robotVelX * timeOfFlight, 
            robotVelY * timeOfFlight
        );

        // E. Create the Virtual Target by subtracting the offset from the real hub
        Translation2d virtualHubTranslation = hubTranslation.minus(inheritedVelocityOffset);

        // F. Calculate the vector to the Virtual Target
        Translation2d turretToVirtualHub = virtualHubTranslation.minus(globalTurretPos);
        
        // G. Save the Virtual Distance so the Shooter can use it!
        m_virtualDistanceToHubMeters = turretToVirtualHub.getNorm();
        SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);

        // ==========================================================
        // 4. --- CALCULATE AIMING ANGLE ---
        // ==========================================================
        
        // Use turretToVirtualHub instead of physical turretToHub!
        Rotation2d turretSetpoint = turretToVirtualHub.getAngle()
            .minus(robotPose.getRotation())
            .minus(Rotation2d.fromDegrees(180)); 

        double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
        
        // Ensure the turret takes the shortest path
        desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
        desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
        
        m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;

        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", desiredTurretRotations);
    }
}