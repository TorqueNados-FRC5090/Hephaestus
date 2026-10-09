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
import frc.robot.FieldZones;

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

    /** Horizontal ball speed used to guess time of flight for shoot-on-the-move (hub shots and passes) */
    private final double kEstimatedShotSpeedMPS = 6.0;

    // --- FIELD (2026 REBUILT, meters) ---
    private final double kFieldLength = 16.54;
    private final double kFieldWidth = 8.07;
    /** Alliance zone line, measured from our own wall. G407: bumpers must be in this zone to shoot at the hub */
    private final double kAllianceZoneDepth = 4.03;
    /** Switch to hub shooting once the robot center is within this distance past the line (bumpers ~0.45m) */
    private final double kEnterShootingMargin = 0.25;
    /** Switch back to passing once the robot center is this far past the line. Gap = no flickering on the line */
    private final double kExitShootingMargin = 0.40;

    // --- PASSING ---
    /** Where passes land, measured from our wall (in front of the tower, inside our alliance zone) */
    private final double kPassLandingDepth = 2.0;
    /** A pass's ground track must stay at least this far from the center of BOTH hubs so it never
     *  clips a hub or the net on its back. Hub is 1.19m square (0.84m center-to-corner). */
    private final double kHubClearanceMeters = 1.0;
    /** Keep pass landing spots this far from the side walls */
    private final double kPassSideMargin = 0.5;

    // --- TRENCH ---
    // The trench roof is 22in up, so a shot fired from under it hits the roof. Hold fire there.
    // Bumpers can already be in our zone while we're still under it on the way back in.
    private final double kTrenchFireMargin = 0.30;

    // --- PHYSICAL TURRET OFFSET ---
    private final double kTurretOffsetXInches = -5; // Backwards
    private final double kTurretOffsetYInches = -6;  // Right

    private final Translation2d m_robotRelativeTurretOffset = new Translation2d(
        Units.inchesToMeters(kTurretOffsetXInches), 
        Units.inchesToMeters(kTurretOffsetYInches)
    );

    // NEW: Flips the direction if the turret is mirroring the target (turns left when target is right)
    private final double kTurretDirectionMultiplier = 1.0; 

    // Which way the turret points at 0 motor rotations: 0 = robot front, 180 = robot back.
    // The turret faces the BACK of the robot, away from the intake.
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
    /** True when the robot is outside our alliance zone, so the turret aims a pass instead of a hub shot */
    private boolean m_isPassing = false;
    /** False when no pass lane clears both hubs (e.g. parked right behind one), so we hold fire */
    private boolean m_passLaneClear = false;
    /** True while the turret is under (or right at the edge of) a trench roof */
    private boolean m_underTrench = false;

    public Turret(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> velocitySupplier, CANBus canbus) {
        this.m_robotPoseSupplier = poseSupplier;
        this.m_robotVelocitySupplier = velocitySupplier; 

        m_turretMotor = new TalonFXS(16, canbus); 
        m_motionMagic = new MotionMagicVoltage(0).withEnableFOC(true);

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

    /** @return true when outside our alliance zone and aiming a pass instead of a hub shot */
    public boolean isPassing() {
        return m_isPassing;
    }

    public double getShootingDistance() {
        if (m_isPassing) {
            return m_distanceToPassTargetMeters;
        }
        return getHubShootingDistance();
    }

    /** Distance for the hub formulas, valid in either mode (autos use it to pre-spin before entering the zone) */
    public double getHubShootingDistance() {
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
        if (m_targetMotorRotations == 0.0 || !m_targetReachable || m_underTrench || (m_isPassing && !m_passLaneClear)) {
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

        // CTRE gives robot-relative speeds; SOTM math works in field coordinates, so rotate them.
        ChassisSpeeds robotRelativeSpeeds = m_robotVelocitySupplier.get();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotPose.getRotation());

        // Where the robot will be facing a moment from now, so the turret leads a spinning robot
        Rotation2d aimHeading = robotPose.getRotation()
            .plus(Rotation2d.fromRadians(robotRelativeSpeeds.omegaRadiansPerSecond * kHeadingLeadSeconds));

        Translation2d globalTurretPos = robotPose.getTranslation()
            .plus(m_robotRelativeTurretOffset.rotateBy(robotPose.getRotation()));

        m_underTrench = FieldZones.isNearTrench(globalTurretPos, kTrenchFireMargin);
        SmartDashboard.putBoolean("Turret/Under_Trench", m_underTrench);

        // 3. --- MODE ---
        // G407: we may only shoot at our hub with bumpers in our alliance zone. Anywhere else we pass.
        double depthFromOurWall = isRed ? kFieldLength - robotPose.getX() : robotPose.getX();
        double shootingLimit = kAllianceZoneDepth + (m_isPassing ? kEnterShootingMargin : kExitShootingMargin);
        m_isPassing = depthFromOurWall > shootingLimit;
        SmartDashboard.putString("Turret/Mode", m_isPassing ? "PASSING" : "SHOOTING");

        // 4. --- HUB TARGET (always computed so autos can spin up before entering the zone) ---
        Translation2d targetCorrectionOffset = new Translation2d(
            Units.inchesToMeters(kTargetCenterOffsetXInches), 
            Units.inchesToMeters(kTargetCenterOffsetYInches)
        );
        Translation2d hubTarget = isRed 
            ? kRedTargetCenter.plus(targetCorrectionOffset) 
            : kBlueTargetCenter.minus(targetCorrectionOffset);
        Translation2d virtualHubTarget = leadTarget(hubTarget, globalTurretPos, fieldSpeeds);

        m_distanceToHubMeters = hubTarget.minus(globalTurretPos).getNorm();
        m_virtualDistanceToHubMeters = virtualHubTarget.minus(globalTurretPos).getNorm();
        SmartDashboard.putNumber("Turret/Distance_To_Hub_Meters", m_distanceToHubMeters);
        SmartDashboard.putNumber("Turret/Virtual_Distance_Meters", m_virtualDistanceToHubMeters);

        // 5. --- PASS TARGET (middle and full field) ---
        Translation2d passTarget = choosePassTarget(globalTurretPos, isRed);
        Translation2d virtualPassTarget = leadTarget(passTarget, globalTurretPos, fieldSpeeds);

        m_distanceToPassTargetMeters = virtualPassTarget.minus(globalTurretPos).getNorm();
        SmartDashboard.putNumber("Turret/Pass_Distance_Meters", m_distanceToPassTargetMeters);
        SmartDashboard.putNumber("Turret/Pass_Target_Y", passTarget.getY());
        SmartDashboard.putBoolean("Turret/Pass_Lane_Clear", m_passLaneClear);

        // 6. --- TURRET SETPOINT ---
        Translation2d aimPoint = m_isPassing ? virtualPassTarget : virtualHubTarget;
        Rotation2d turretSetpoint = aimPoint.minus(globalTurretPos).getAngle()
            .minus(aimHeading)
            .minus(kTurretZeroOffset); 
        double desiredTurretRotations = turretSetpoint.getRadians() / (2 * Math.PI);

        // APPLY DIRECTION FIX
        desiredTurretRotations *= kTurretDirectionMultiplier;

        desiredTurretRotations = Math.IEEEremainder(desiredTurretRotations, 1.0);
        m_targetReachable = Math.abs(desiredTurretRotations) <= kMaxTurretRotations;
        desiredTurretRotations = MathUtil.clamp(desiredTurretRotations, -kMaxTurretRotations, kMaxTurretRotations);
        
        m_targetMotorRotations = desiredTurretRotations * kTurretGearRatio;

        SmartDashboard.putNumber("Turret/Target_Motor_Rots", m_targetMotorRotations);
        SmartDashboard.putNumber("Turret/Target_Turret_Rots", m_targetMotorRotations / kTurretGearRatio);
        SmartDashboard.putBoolean("Turret/Target_Reachable", m_targetReachable);
    }

    /** Shoot-on-the-move: the ball keeps the robot's velocity, so aim at a point shifted against it */
    private Translation2d leadTarget(Translation2d target, Translation2d turretPos, ChassisSpeeds fieldSpeeds) {
        double timeOfFlight = target.minus(turretPos).getNorm() / kEstimatedShotSpeedMPS;
        return target.minus(new Translation2d(
            fieldSpeeds.vxMetersPerSecond * timeOfFlight,
            fieldSpeeds.vyMetersPerSecond * timeOfFlight));
    }

    /** Picks a landing spot in our alliance zone whose straight-line ground track clears both hubs
     *  (and the nets on their backs). Prefers passing straight down the robot's own lane. */
    private Translation2d choosePassTarget(Translation2d turretPos, boolean isRed) {
        double landingX = isRed ? kFieldLength - kPassLandingDepth : kPassLandingDepth;
        double minY = kPassSideMargin;
        double maxY = kFieldWidth - kPassSideMargin;
        double preferredY = MathUtil.clamp(turretPos.getY(), minY, maxY);
        double hubY = kBlueTargetCenter.getY();
        boolean onUpperSide = turretPos.getY() >= hubY;

        Translation2d best = null;
        double bestCost = Double.MAX_VALUE;
        for (double y = minY; y <= maxY + 1e-9; y += 0.05) {
            Translation2d candidate = new Translation2d(landingX, y);
            if (!clearsBothHubs(turretPos, candidate)) {
                continue;
            }
            // Tiny penalty for crossing to the other side of the hubs, so a centered robot doesn't flip-flop
            double cost = Math.abs(y - preferredY) + ((y >= hubY) == onUpperSide ? 0.0 : 0.01);
            if (cost < bestCost) {
                bestCost = cost;
                best = candidate;
            }
        }

        m_passLaneClear = best != null;
        if (best == null) {
            // No clear lane (robot is tucked right behind a hub). Aim down our side anyway but hold fire.
            best = new Translation2d(landingX, onUpperSide ? maxY : minY);
        }
        return best;
    }

    private boolean clearsBothHubs(Translation2d from, Translation2d to) {
        return distanceToSegment(kBlueTargetCenter, from, to) >= kHubClearanceMeters
            && distanceToSegment(kRedTargetCenter, from, to) >= kHubClearanceMeters;
    }

    /** Shortest distance from point p to the line segment a-b */
    private static double distanceToSegment(Translation2d p, Translation2d a, Translation2d b) {
        Translation2d ab = b.minus(a);
        double lengthSquared = ab.getX() * ab.getX() + ab.getY() * ab.getY();
        double t = 0.0;
        if (lengthSquared > 0.0) {
            Translation2d ap = p.minus(a);
            t = MathUtil.clamp((ap.getX() * ab.getX() + ap.getY() * ab.getY()) / lengthSquared, 0.0, 1.0);
        }
        return p.getDistance(a.plus(ab.times(t)));
    }
}
