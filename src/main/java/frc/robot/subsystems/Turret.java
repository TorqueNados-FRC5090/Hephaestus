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
    private double m_targetMotorRotations = 0.0;
    private boolean m_canSeeTarget = false;

    public Turret(Supplier<Pose2d> poseSupplier, AprilTagFieldLayout atLayout) {
        this.m_robotPoseSupplier = poseSupplier;
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


    /**
     * Commands the motor to snap to the target. 
     */
    public void alignToHub() {
        if (!m_canSeeTarget) return;
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    //Command that returns the Turret to the zero point when not in use
    public void goToZero() {
        m_turretMotor.setControl(m_motionMagic.withPosition(0));
    }

    /** Checks the turret if it is a the setpoint */

    public boolean isTurretReady(){
        // If we aren't trying to shoot, the shooter isn't "ready"
        if (m_targetMotorRotations == 0.0) {
            return false;
        }

        // Get the current speed
        double currentpos = m_turretMotor.getPosition().refresh().getValueAsDouble();

        //check the postion to 0.2 of a rotation
        return Math.abs(currentpos - m_targetMotorRotations) <= 0.2;
    }




    /**
     * Call this when pressing the shoot button! 
     * It intelligently decides to pass if past midfield, or shoot at the hub if close.
     */
    public void passOrShoot() {
        Pose2d robotPose = m_robotPoseSupplier.get();
        boolean isRed = isRedAlliance();
        
        // FRC field is ~16.54 meters long. Midfield is exactly half.
        double fieldMidpointX = 16.54 / 2.0; 
        
        // Check if we are past the midline into the opponent's zone (or middle)
        boolean inOpponentOrMidZone = isRed ? (robotPose.getX() <= fieldMidpointX) : (robotPose.getX() >= fieldMidpointX);

        if (inOpponentOrMidZone) {
            // Target the center of our alliance wall (X=0 for Blue, X=16.54 for Red)
            // We use the robot's current Y coordinate to pass straight backwards
            Translation2d passTarget = new Translation2d(isRed ? 16.54 : 0.0, robotPose.getY());

            Translation2d globalTurretPos = robotPose.getTranslation()
                .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

            Translation2d turretToPassTarget = passTarget.minus(globalTurretPos);
            Rotation2d turretSetpoint = turretToPassTarget.getAngle().minus(robotPose.getRotation());

            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_turretMotor.setControl(m_motionMagic.withPosition(desiredTurretRotations * kTurretGearRatio));
        } else {
            // If we are safe in our own zone, aim normally at the hub
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

        // 2. --- LIVE RANGEFINDER MATH ---
        boolean isRed = isRedAlliance();
        int targetTag = isRed ? 10 : 26; 
        Optional<Pose3d> tagPoseOpt = m_atLayout.getTagPose(targetTag);
        
        if (tagPoseOpt.isEmpty()) {
            m_canSeeTarget = false;
            m_distanceToHubMeters = 0.0; 
            SmartDashboard.putString("Turret/STATUS", "ERROR: Tag " + targetTag + " not found!");
            SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", 0.0);
            return; 
        } 
        
        m_canSeeTarget = true;
        SmartDashboard.putString("Turret/STATUS", "Tracking Tag " + targetTag);
        
        Pose2d hubPose = tagPoseOpt.get().toPose2d();
        Translation2d hubTranslation;
        
        // APPLYING THE CENTER HUB OFFSETS HERE
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

        Translation2d turretToHub = hubTranslation.minus(globalTurretPos);
        
        m_distanceToHubMeters = turretToHub.getNorm();

        // 3. --- PUSH LIVE DISTANCE TO DASHBOARD ---
        SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);

        // 4. --- CALCULATE AIMING ANGLE ---
        // (Target ANgle - Robot Angle - 180 degrees)
        Rotation2d turretSetpoint = turretToHub.getAngle().minus(robotPose.getRotation()).minus(Rotation2d.fromDegrees(180)); 

        double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
        desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
        m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;

        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", desiredTurretRotations);
    }
}