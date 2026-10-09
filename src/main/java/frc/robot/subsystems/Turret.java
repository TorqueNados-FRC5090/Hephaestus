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
    // Hub centers from the official 2026 REBUILT AprilTag layout (middle of tags 18-27 / 2-11).
    // These used to be the alliance walls (x = 0 / 16.54), which made the shooter spin
    // faster as the robot got CLOSER to the hub (it was getting farther from the wall).
    private final Translation2d kBlueTargetCenter = new Translation2d(4.625, 4.035);
    private final Translation2d kRedTargetCenter = new Translation2d(11.915, 4.035);

    // The 3/20 shooter + hood formulas were fit using the distance to the hub's front tag
    // (tag 26 blue / tag 10 red), which sits this far in front of the hub center.
    // We aim at the center, but subtract this so the formulas still get the distance they expect.
    private final double kHubFaceToCenterMeters = 0.604;

    // --- SOTM AIM LEAD ---
    // How far ahead (seconds) to predict the robot's heading so the turret doesn't lag
    // behind when the robot spins. 0 = off. Raise if it still trails, lower if it overshoots.
    private final double kHeadingLeadSeconds = 0.10;

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -5; // Backwards
    private final double kTurretOffsetYInches = -6;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches), 
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // NEW: Flips the direction if the turret is mirroring the target (turns left when target is right)
    private final double kTurretDirectionMultiplier = 1.0; 

    // Turret zero faces the BACK of the robot, so this is 180 (same as the March code that aimed at tag 26/10).
    // It was set to 0 while the target was the alliance wall; the two mistakes cancelled out on the
    // field centerline only, which is why off-center shots missed.
    private final Rotation2d kTurretZeroOffset = Rotation2d.fromDegrees(180);

    // --- TARGET OFFSET CORRECTION ---
    private final double kTargetCenterOffsetXInches = 0.0; 
    private final double kTargetCenterOffsetYInches = 0.0;  

    // --- Mechanical Constants ---
    private final double kTurretRingTeeth = 200.0; 
    private final double kEncoderGearTeeth = 16.0; 
    private final double kTurretGearRatio = kTurretRingTeeth / kEncoderGearTeeth; 
    private final double kMaxTurretRotations = 0.30; //0.48

    // --- LIVE STATE VARIABLES ---
    public double m_distanceToHubMeters = 0.0;
    public double m_distanceToPassTargetMeters = 0.0;
    public double m_virtualDistanceToHubMeters = 0.0; 
    private double m_targetMotorRotations = 0.0;
    /** False when the target is outside the turret's travel, so we don't fire while parked at the limit */
    private boolean m_targetReachable = false;

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
        // Volts per motor rot/s. Without this the turret only moves once it's already behind
        // (pure kP), which is most of the lag. ~12V / Minion free speed.
        config.Slot0.kV = 0.1;

        // Old values: cruise 600 (faster than the motor can spin), accel 60 (took ~2s to reach speed).
        // Acceleration was the real limit. Turn accel down if the turret slams or skips teeth.
        config.MotionMagic.MotionMagicCruiseVelocity = 90.0;
        config.MotionMagic.MotionMagicAcceleration = 400.0;
        config.MotionMagic.MotionMagicJerk = 4000.0;

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
        // Shooter/hood formulas expect distance to the hub's front tag, not its center
        return Math.max(0.0, m_virtualDistanceToHubMeters - kHubFaceToCenterMeters); 
    }

    public void alignToHub() {
        m_turretMotor.setControl(m_motionMagic.withPosition(m_targetMotorRotations));
    }

    public void goToZero() {
        m_turretMotor.setControl(m_motionMagic.withPosition(0));
    }

    public boolean isTurretReady(){
        if (m_targetMotorRotations == 0.0 || !m_targetReachable) {
            return false;
        }
        double currentpos = m_turretMotor.getPosition().refresh().getValueAsDouble();
        return Math.abs(currentpos - m_targetMotorRotations) <= 0.2;
    }

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
        double fieldWidth = 8.07; 
        double fieldMidpointX = fieldLength / 2.0; 
        double hubCenterY = fieldWidth / 2.0; 

        // CTRE gives robot-relative speeds; SOTM math works in field coordinates, so rotate them.
        ChassisSpeeds robotRelativeSpeeds = m_robotVelocitySupplier.get();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

        // Where the robot will be facing a moment from now, so the turret leads a spinning robot
        Rotation2d aimHeading = robotPose.getRotation()
            .plus(Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond * kHeadingLeadSeconds));
        
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

            Rotation2d turretSetpoint = turretToPassTarget.getAngle()
                .minus(aimHeading)
                .minus(kTurretZeroOffset); 
            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);

            // APPLY DIRECTION FIX
            desiredTurretRotations *= kTurretDirectionMultiplier;

            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            m_targetReachable = Math.abs(desiredTurretRotations) <= kMaxTurretRotations;
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

            double robotVelX = fieldSpeeds.vxMetersPerSecond;
            double robotVelY = fieldSpeeds.vyMetersPerSecond;
            double kEstimatedShotSpeedMPS = 6.0; 

            double timeOfFlight = m_distanceToHubMeters / kEstimatedShotSpeedMPS;
            Translation2d inheritedVelocityOffset = new Translation2d(robotVelX * timeOfFlight, robotVelY * timeOfFlight);
            Translation2d virtualTargetTranslation = finalTargetTranslation.minus(inheritedVelocityOffset);
            Translation2d turretToVirtualTarget = virtualTargetTranslation.minus(globalTurretPos);
            
            m_virtualDistanceToHubMeters = turretToVirtualTarget.getNorm();
            SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);

            Rotation2d turretSetpoint = turretToVirtualTarget.getAngle()
                .minus(aimHeading)
                .minus(kTurretZeroOffset); 
            double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);

            // APPLY DIRECTION FIX
            desiredTurretRotations *= kTurretDirectionMultiplier;

            desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
            m_targetReachable = Math.abs(desiredTurretRotations) <= kMaxTurretRotations;
            desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
            
            m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;
        }

        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", m_targetMotorRotations / kTurretGearRatio);
        SmartDashboard.putBoolean("Turret/Target_Reachable", m_targetReachable);
    }
}