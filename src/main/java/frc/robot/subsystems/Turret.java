package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
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
    private final Translation2d kBlueTargetCenter = new Translation2d(0.0, 4.105); 
    private final Translation2d kRedTargetCenter = new Translation2d(16.54, 4.105); 

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -4.5; // Backwards
    private final double kTurretOffsetYInches = -3.375;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches), 
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // --- TARGET OFFSET CORRECTION ---
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

    public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier, CANBus canbus) {
        this.m_robotPoseSupplier = poseSupplier;
        this.m_robotVelocitySupplier = velocitySupplier; 

        m_turretMotor = new TalonFXS(16, canbus); 
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

    /** Legacy command hook (Can be removed if not used manually) */
    public void alignToHub() {
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    public void goToZero() {
        m_turretMotor.setControl(m_motionMagic.withPosition(0));
    }

    /** Accurately checks if the turret is at its calculated setpoint for EITHER passing or shooting. */
    public boolean isTurretReady(){
        if (m_targetMotorRotations == 0.0) {
            return false;
        }
        double currentpos = m_turretMotor.getPosition().refresh().getValueAsDouble();
        return Math.abs(currentpos - m_targetMotorRotations) <= 0.2;
    }

    /** * Runs continuously as the default command. 
     * Simply tells the motor to move to whatever setpoint periodic() calculated.
     */
    public void passOrShoot() {
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    private boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    @Override
    public void periodic() {
        // 1. --- LIVE MOTOR DATA ---
        double currentMotorRotations = m_turretMotor.getPosition().refresh().getValueAsDouble();
        SmartDashboard.putNumber("Turret/Current_Motor_Rots", currentMotorRotations);
        SmartDashboard.putNumber("Turret/Current_Turret_Rots", currentMotorRotations / kTurretGearRatio);

        // 2. --- FIELD VARIABLES ---
        Pose2d robotPose = m_robotPoseSupplier.get();
        boolean isRed = isRedAlliance();
        
        double fieldLength = 16.54;
        double fieldWidth = 8.21; 
        double fieldMidpointX = fieldLength / 2.0; 
        double hubCenterY = fieldWidth / 2.0; 
        
        boolean inOpponentOrMidZone = isRed ? (robotPose.getX() <= fieldMidpointX) : (robotPose.getX() >= fieldMidpointX);

        Translation2d globalTurretPos = robotPose.getTranslation()
            .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

        // ==========================================================
        // 3. --- CALCULATE MASTER TARGET BASED ON ZONE ---
        // ==========================================================
        if (inOpponentOrMidZone) {
            // -----------------------------
            // A. PASSING MODE CALCULATION
            // -----------------------------
            SmartDashboard.putString("Turret/Mode", "PASSING");
            
            double passTargetX = isRed ? fieldLength : 0.0;
            double passTargetY = robotPose.getY();
            double dangerZoneClearanceMeters = 1.5; 

            // Shift target to avoid hitting the hub structure
            if (Math.abs(robotPose.getY() - hubCenterY) < dangerZoneClearanceMeters) {
                passTargetY = (robotPose.getY() >= hubCenterY) 
                    ? hubCenterY + dangerZoneClearanceMeters 
                    : hubCenterY - dangerZoneClearanceMeters;
            }
            
            passTargetY = MathUtil.clamp(passTargetY, 0.5, fieldWidth - 0.5);
            Translation2d passTarget = new Translation2d(passTargetX, passTargetY);
            Translation2d turretToPassTarget = passTarget.minus(globalTurretPos);
            
            m_distanceToPassTargetMeters = turretToPassTarget.getNorm();
            SmartDashboard.putNumber("Turret/Pass_Distance_Meters", m_distanceToPassTargetMeters);
            SmartDashboard.putNumber("Turret/Pass_Target_Y", passTargetY);

            // Compute desired rotations for passing
            Rotation2d turretSetpoint = turretToPassTarget.getAngle().minus(robotPose.getRotation()).minus(Rotation2d.fromDegrees(180));
            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;

        } else {
            // -----------------------------
            // B. SHOOTING MODE CALCULATION
            // -----------------------------
            SmartDashboard.putString("Turret/Mode", "SHOOTING");

            Translation2d rawTargetTranslation = isRed ? kRedTargetCenter : kBlueTargetCenter;
            Translation2d targetCorrectionOffset = new Translation2d(
                Units.inchesToMeters(kTargetCenterOffsetXInches), 
                Units.inchesToMeters(kTargetCenterOffsetYInches)
            );

            Translation2d finalTargetTranslation = isRed 
                ? rawTargetTranslation.plus(targetCorrectionOffset) 
                : rawTargetTranslation.minus(targetCorrectionOffset);
                
            Translation2d turretToTarget = finalTargetTranslation.minus(globalTurretPos);
            m_distanceToHubMeters = turretToTarget.getNorm();
            SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);

            // Shoot-on-the-move inherited velocity calculations
            var speeds = m_robotVelocitySupplier.get();
            double robotVelX = speeds.vxMetersPerSecond;
            double robotVelY = speeds.vyMetersPerSecond;
            double kEstimatedShotSpeedMPS = 6.0; // TUNE THIS ON THE FIELD

            double timeOfFlight = m_distanceToHubMeters / kEstimatedShotSpeedMPS;
            Translation2d inheritedVelocityOffset = new Translation2d(robotVelX * timeOfFlight, robotVelY * timeOfFlight);
            Translation2d virtualTargetTranslation = finalTargetTranslation.minus(inheritedVelocityOffset);
            Translation2d turretToVirtualTarget = virtualTargetTranslation.minus(globalTurretPos);
            
            m_virtualDistanceToHubMeters = turretToVirtualTarget.getNorm();
            SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);

            // Compute desired rotations for shooting
            Rotation2d turretSetpoint = turretToVirtualTarget.getAngle().minus(robotPose.getRotation()).minus(Rotation2d.fromDegrees(180)); 
            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);
            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;
        }

        // Post final requested values to Dashboard
        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", m_targetMotorRotations / kTurretGearRatio);
    }
}