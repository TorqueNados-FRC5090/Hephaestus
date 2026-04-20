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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
    private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;

    // --- ABSOLUTE FIELD TARGETS (THE GRID/HUB) ---
    // TUNE THESE: These are the exact X/Y coordinates of your targets in meters.
    // X=0 is the Blue Alliance Wall, X=16.54 is the Red Alliance Wall.
    // Y=4.105 is the exact center width of the field. Adjust Y if the target is off-center!
    private final Translation2d kBlueTargetCenter = new Translation2d(0.0, 4.105); 
    private final Translation2d kRedTargetCenter = new Translation2d(16.54, 4.105); 

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -4; // Backwards
    private final double kTurretOffsetYInches = -5;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches), 
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // --- TARGET OFFSET CORRECTION ---
    // Tune these if your physical mechanism consistently shoots left/right of the absolute center
    private final double kTargetCenterOffsetXInches = 0.0; 
    private final double kTargetCenterOffsetYInches = -2.0;  

    // --- Mechanical Constants ---
    private final double kTurretRingTeeth = 200.0; 
    private final double kEncoderGearTeeth = 16.0; 
    private final double kTurretGearRatio = kTurretRingTeeth / kEncoderGearTeeth; 
    private final double kMaxTurretRotations = 0.48; 

    // --- LIVE STATE VARIABLES ---
    public double m_distanceToHubMeters = 0.0;
    public double m_distanceToPassTargetMeters = 0.0;
    public double m_virtualDistanceToHubMeters = 0.0; 
    private double m_targetMotorRotations = 0.0;

    // NOTE: We removed the AprilTagFieldLayout from the constructor!
    public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier) {
        this.m_robotPoseSupplier = poseSupplier;
        this.m_robotVelocitySupplier = velocitySupplier; 

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

    public double getShootingDistance() {
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            return m_distanceToPassTargetMeters;
        }
        return m_virtualDistanceToHubMeters; 
    }

    /** Commands the motor to snap to the target. */
    public void alignToHub() {
        // Because we use odometry, we never lose sight of the target!
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    public void goToZero() {
        m_turretMotor.setControl(m_motionMagic.withPosition(0));
    }

    public boolean isTurretReady(){
        if (m_targetMotorRotations == 0.0) {
            return false;
        }
        double currentpos = m_turretMotor.getPosition().refresh().getValueAsDouble();
        return Math.abs(currentpos - m_targetMotorRotations) <= 0.2;
    }

    public void passOrShoot() {
        Pose2d robotPose = m_robotPoseSupplier.get();
        boolean isRed = isRedAlliance();
        
        double fieldLength = 16.54;
        double fieldWidth = 8.21; 
        double fieldMidpointX = fieldLength / 2.0; 
        double hubCenterY = fieldWidth / 2.0; 
        
        boolean inOpponentOrMidZone = isRed ? (robotPose.getX() <= fieldMidpointX) : (robotPose.getX() >= fieldMidpointX);

        if (inOpponentOrMidZone) {
            // --- PASSING & AVOIDANCE LOGIC ---
            double passTargetX = isRed ? fieldLength : 0.0;
            double passTargetY = robotPose.getY();
            double dangerZoneClearanceMeters = 1.5; 

            if (Math.abs(robotPose.getY() - hubCenterY) < dangerZoneClearanceMeters) {
                if (robotPose.getY() >= hubCenterY) {
                    passTargetY = hubCenterY + dangerZoneClearanceMeters;
                } else {
                    passTargetY = hubCenterY - dangerZoneClearanceMeters;
                }
            }
            
            passTargetY = MathUtil.clamp(passTargetY, 0.5, fieldWidth - 0.5);
            Translation2d passTarget = new Translation2d(passTargetX, passTargetY);
            Translation2d globalTurretPos = robotPose.getTranslation().plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));
            Translation2d turretToPassTarget = passTarget.minus(globalTurretPos);
            
            m_distanceToPassTargetMeters = turretToPassTarget.getNorm();
            SmartDashboard.putNumber("Turret/Pass_Distance_Meters", m_distanceToPassTargetMeters);

            Rotation2d turretSetpoint = turretToPassTarget.getAngle().minus(robotPose.getRotation()).minus(Rotation2d.fromDegrees(180));
            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_turretMotor.setControl(m_motionMagic.withPosition(desiredTurretRotations * kTurretGearRatio));
            
            SmartDashboard.putString("Turret/Mode", "PASSING");
            SmartDashboard.putNumber("Turret/Pass_Target_Y", passTargetY);
        } else {
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

        // ==========================================================
        // 2. --- ODOMETRY-BASED TARGET ACQUISITION ---
        // ==========================================================
        boolean isRed = isRedAlliance();
        
        // Grab the absolute field coordinate of our target
        Translation2d rawTargetTranslation = isRed ? kRedTargetCenter : kBlueTargetCenter;
        
        // Apply physical tuning offsets
        Translation2d targetCorrectionOffset = new Translation2d(
            Units.inchesToMeters(kTargetCenterOffsetXInches), 
            Units.inchesToMeters(kTargetCenterOffsetYInches)
        );

        Translation2d finalTargetTranslation = isRed 
            ? rawTargetTranslation.plus(targetCorrectionOffset) 
            : rawTargetTranslation.minus(targetCorrectionOffset);
            
        Pose2d robotPose = m_robotPoseSupplier.get(); 
        Translation2d globalTurretPos = robotPose.getTranslation()
            .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

        // Calculate physical straight-line distance to the coordinate
        Translation2d turretToTarget = finalTargetTranslation.minus(globalTurretPos);
        m_distanceToHubMeters = turretToTarget.getNorm();
        SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);

        // ==========================================================
        // 3. --- SHOOT ON THE MOVE (VIRTUAL TARGET) MATH ---
        // ==========================================================
        var speeds = m_robotVelocitySupplier.get();
        double robotVelX = speeds.vxMetersPerSecond;
        double robotVelY = speeds.vyMetersPerSecond;

        double kEstimatedShotSpeedMPS = 6.0; // TUNE THIS

        double timeOfFlight = m_distanceToHubMeters / kEstimatedShotSpeedMPS;
        Translation2d inheritedVelocityOffset = new Translation2d(robotVelX * timeOfFlight, robotVelY * timeOfFlight);
        Translation2d virtualTargetTranslation = finalTargetTranslation.minus(inheritedVelocityOffset);
        Translation2d turretToVirtualTarget = virtualTargetTranslation.minus(globalTurretPos);
        
        m_virtualDistanceToHubMeters = turretToVirtualTarget.getNorm();
        SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);

        // ==========================================================
        // 4. --- CALCULATE AIMING ANGLE ---
        // ==========================================================
        Rotation2d turretSetpoint = turretToVirtualTarget.getAngle()
            .minus(robotPose.getRotation())
            .minus(Rotation2d.fromDegrees(180)); 

        double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
        
        desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
        desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
        
        m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;

        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", desiredTurretRotations);
    }
}