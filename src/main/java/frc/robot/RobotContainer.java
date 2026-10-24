// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType; // Added for safety clamp
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // Added for Passing
import java.util.Map;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.EvilIntakePiece;
import frc.robot.commands.theYappy;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EvilIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.RollerSystem;
import frc.robot.subsystems.Turret;
import frc.robot.wrappers.Limelight;

public class RobotContainer {
    // --- EXTRA VARIABLES START ---
    public final CANBus upper = new CANBus("Upper");
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    // 0.43 keeps the same spin feel as before kSpeedAt12Volts was corrected (it was 0.75 with a 2x-too-high
    // top speed, which really gave ~0.43 rot/s). Raise it if the drivers want faster turning.
    private double MaxAngularRate = RotationsPerSecond.of(0.43).in(RadiansPerSecond); 
    // --- EXTRA VARIABLES END ---

    // --- SWERVE DRIVE VARIABLES START ---
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final Limelight limelight = new Limelight("limelight");
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // --- SWERVE DRIVE VARIABLES END ---

    // --- TURRET VARIABLES START ---
    // Hood is held down whenever the robot is in, or about to drive into, a trench (a raised hood breaks there)
    public final Hood hood = new Hood(upper, () -> {
        var state = drivetrain.getState();
        return FieldZones.robotHeadingIntoTrench(state.Pose,
            ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()),
            Hood.kTrenchReachMeters, Hood.kTrenchLookaheadSeconds);
    });
    // Rack: leader 11 (positive = out) + follower 14 (opposed), both Talon FXS + Vortex. Wheels: 12 (Talon FX).
    // 10/23: second intake wheel motor added (CAN 18, Kraken X60), opposed-Follower of the leader wheel (12)
    public final EvilIntake evilIntake = new EvilIntake(11, 14, 12, 18, upper);
    public final Shooter shooter = new Shooter(upper);
    public final RollerSystem rollersystem = new RollerSystem(upper);
    
    // SOTM UPDATE: Passing Pose and Speeds (Removed FieldLayout)
    public final Turret turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> drivetrain.getState().Speeds,
            upper
        );

    final AutonContainer auton = new AutonContainer(this); 
    final SendableChooser<Command> autonChooser = auton.buildAutonChooser();
    // --- TURRET VARIABLES END ---

    // --- PASSING INTERPOLATION MAPS ---
    private final InterpolatingDoubleTreeMap m_passRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passHoodMap = new InterpolatingDoubleTreeMap();

    // --- BROWNOUT RUMBLE ---
    /** Battery voltage where the controller starts to rumble. Full rumble at the brownout voltage (6.75V). */
    private static final double kRumbleStartVolts = 7.5;
    /** How fast the rumble fades after a dip (per 20ms loop). Dips last milliseconds, so this lets the driver feel them. */
    private static final double kRumbleDecayPerLoop = 0.03;
    private double m_brownoutRumble = 0.0;

    // Each ball pulls the flywheel down, and with 2 brass flywheels removed it drops further.
    // Keep feeding for a moment after "ready" goes false so the rollers don't stutter between balls.
    private final Debouncer m_readyDebouncer = new Debouncer(0.15, DebounceType.kFalling);

    // EXPLANATION: This is the Constructor. It runs once when the robot boots up.
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        configureBindings();
        
        // Populate the passing maps with your field-length data
        // 8.27m = center of the field, half of 16.54m -- 10/22 field test: passing from there was too hot at
        // 45 RPS. Dropped to 38 as a first guess (no real data point yet for how much too hot); if it's still
        // too strong or now too weak, say by how much and this can get tuned closer.
        m_passRpmMap.put(8.27, 38.0);
        m_passRpmMap.put(16.54, 60.0);
        m_passHoodMap.put(8.27, -2.2);
        m_passHoodMap.put(16.54, -2.2);
    }

    /** @return Whether the robot is on the red alliance or not. */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }

    // EXPLANATION: This wires your physical Xbox controller to your robot's code commands.
    private void configureBindings() {
        
        // DRIVETRAIN DEFAULT COMMAND (Includes Trigger Slow-Mode)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Slow down while actually shooting on the move (RT held past 50%, or Y -- shoot + keep intaking).
                // B (failsafe) isn't included: it zeros the turret and is meant for standing still anyway.
                // 0.4->0.3 (10/23): 044730 log, the driving right at the end (no buttons held, Michel just easing
                // off the stick) sat around 1.0-1.8 m/s and he said he liked that pace. 0.3x of the 5.85 m/s free
                // speed caps it at 1.76 m/s, right in that range. Raise/lower this if the shot-on-the-move pace
                // feels off from that.
                boolean shootingOnTheMove = joystick.getRightTriggerAxis() > 0.5 || joystick.y().getAsBoolean();
                double slowModeMultiplier = shootingOnTheMove ? 0.3 : 1.0;

                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowModeMultiplier)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowModeMultiplier)
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowModeMultiplier);
            })
        );

        // --- CONTINUOUS TRACKING ---
        // This makes the turret run passOrShoot() continuously whenever no other command is using it.
        turret.setDefaultCommand(turret.run(turret::passOrShoot));
        // Hood sits down (safe for the trench) whenever nothing is shooting
        hood.setDefaultCommand(hood.stowCommand());

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // CONTROLLER BUTTONS
        // Y: same as RT (just shoot) plus the gentle intake agitation, so fuel keeps feeding while shooting
        joystick.y().whileTrue(Commands.parallel(fullShootCommand(), evilIntake.agitate(EvilIntakePosition.in)));
        joystick.b().whileTrue(failsafeShoot());
        joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        
        
       // joystick.leftBumper().whileTrue(new EvilIntakePiece(evilIntake, EvilIntakePosition.out))
       // .and(joystick.leftBumper().whileTrue(rollersystem.roll(100)));
        joystick.leftBumper().whileTrue(
            Commands.parallel(
                new EvilIntakePiece(evilIntake, EvilIntakePosition.out),
                rollersystem.intakeFloor()   // floor only; the tunnels only run when shooting
            )
            );

        
        // Unjam: floor + lower + upper tunnel all spin backwards while held
        joystick.rightBumper().whileTrue(rollersystem.otherUnjam());
        // Eject: everything backwards like unjam, plus the intake goes out with its wheels reversed to spit fuel out
        joystick.leftTrigger().whileTrue(Commands.parallel(rollersystem.otherUnjam(), evilIntake.eject()));
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 
        // Re-zero the intake against its extension stop (after a big hit, or if it says SIDES_OUT_OF_SYNC)
        joystick.povUp().onTrue(evilIntake.requestHomeCommand());
        // Shop testing only: pretend we're straight in front of the hub (disabled when connected to a real match)
        joystick.back().and(() -> !DriverStation.isFMSAttached()).onTrue(shopTestPose());
        
        // RT: just shoot -- fires the shooter, moves the hood, slows the chassis, intake untouched
        joystick.rightTrigger().whileTrue(fullShootCommand());
        // B (failsafe shot) still doesn't agitate -- it's meant to be a quick "just get a shot off" button,
        // same spirit as RT. Y is the one that agitates (above). evilIntake.agitate() is also used by autos
        // (AutonContainer's "IntakeAgitate").

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // EXPLANATION: The FMS calls this right before Auto starts to ask what sequence to run.
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /** This will coordinate all necessary subsystems and only shoot when they all report readiness.
     *  "Just shoot" (RT): the intake is left alone, same as always. */
    public Command fullShootCommand() {
        return fullShootCommand(false);
    }

    /** @param alsoIntake true = shoot on the move AND keep the intake deployed + collecting fuel the whole
     *  time (Y) -- the floor keeps pulling fuel in regardless of what the feed tunnels are doing.
     *  false = shooting only, intake untouched (RT, "just shoot"). */
    public Command fullShootCommand(boolean alsoIntake) {
        Command shootAndFeed = new ParallelCommandGroup(
            shooter.shoot(() -> calculateOptimalShooterRPS()),
            // Notice MoveTurret is gone! The default command we set above handles aiming.

            // Command C: Shoot only when the other subsystems are ready.
            new theYappy(rollersystem, () -> readyToShoot(), () -> logShot("teleop"), alsoIntake),
            hood.hoodgo(() -> calculateOptimalHoodAngle())
        );
        // EvilIntake is a separate subsystem from the rollers (theYappy above), so this runs alongside it fine
        return alsoIntake
            ? shootAndFeed.alongWith(new EvilIntakePiece(evilIntake, EvilIntakePosition.out))
            : shootAndFeed;
    }

    /** Auton-only shoot: never passes. Spins the shooter + hood for the hub the whole time (so it's
     *  already up to speed when the robot drives back into our zone) and only feeds while we're
     *  legally in our alliance zone and everything is ready. Works while driving (shoot on the move). */
    public Command autoShootCommand() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> hubShooterRPS(turret.getHubShootingDistance())),
            new theYappy(rollersystem, () -> !turret.isPassing() && readyToShoot(), () -> logShot("auto")),
            hood.hoodgo(() -> hubHoodAngle(turret.getHubShootingDistance()))
        );
    }

    /** Gets the shooter and hood to hub speed/angle without feeding anything */
    public Command spinUpCommand() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> hubShooterRPS(turret.getHubShootingDistance())),
            hood.hoodgo(() -> hubHoodAngle(turret.getHubShootingDistance()))
        );
    }

    /** FIXED SHOT (B button). Ignores vision, pose and turret aiming: turret parked at zero (straight back),
     *  fixed 23 RPS, feeds as soon as the flywheel is up to speed. Use it if the turret gets misaligned. */
    /** Failsafe shoot that does not coordinate and instead sets everything to the minimum it can to shoot without an Apriltag. Should just shoot forward.  */
    public Command failsafeShoot() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> 23), 
            // The failsafe safely interrupts the default command to zero the turret, then resumes tracking when released
            turret.run(() -> turret.goToZero()),
            new theYappy(rollersystem, () -> shooter.isShooterReady(2), () -> logShot("fixed shot"))
        );
    }

    /** This will coordinate all necessary subsystems and only shoot when they all report readiness */
    public Command testTheStupids() {
        return new ParallelCommandGroup(
            // Command C: Shoot only when the other subsystems are ready.
            new theYappy(rollersystem, () -> readyToShoot())
        );
    }

    // EXPLANATION: Calculates wheel speed based on SOTM distance.
    public double calculateOptimalShooterRPS() {
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();

        // 2. Override if Passing
        if (turret.isPassing()) {
            return m_passRpmMap.get(targetDist);
        }
        
        return hubShooterRPS(targetDist);
    }

    /** New Equation 3/20/26 (For Hub Shooting) */
    private double hubShooterRPS(double targetDist) {
        return (25 + 0.697 * targetDist + 0.243 * Math.pow(targetDist, 2)); //20.9 -> 25
    }

    // EXPLANATION: Calculates hood deflection based on SOTM distance.
    public double calculateOptimalHoodAngle() {
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();

        // 2. Override if Passing
        if (turret.isPassing()) {
            return clampHood(m_passHoodMap.get(targetDist));
        } 
        return hubHoodAngle(targetDist);
    }

    /** New Equation 3/20/26 (For Hub Shooting) */
    private double hubHoodAngle(double targetDist) {
        double optimal = 0;
        if (targetDist >= 2.2) {
            optimal = (1 - (0.463 * targetDist));
        }
        return clampHood(optimal);
    }

    private double clampHood(double optimal) {

        // --- NEW SAFETY LIMIT ---
        // Replace -3.0 with the absolute maximum negative value your hood can physically go.
        // Replace 0.0 with your resting/minimum position.
        double maxExtension = -2.7734375; 
        double minExtension = -0.12890625;  

        // MathUtil.clamp ensures 'optimal' never goes below maxExtension or above minExtension
        optimal = MathUtil.clamp(optimal, maxExtension, minExtension);

        SmartDashboard.putNumber("Optimal Hood Angle", optimal);
        return optimal;
    }

    // Buttons worth logging by name (matches the printed controller card in /workspace/controller-card).
    // Raw button bits are already in the .wpilog (every DS axis/button, automatically), but decoding "button 5
    // on joystick 0" back into "LB" takes the controller's mapping and isn't readable on its own -- this logs
    // the actual name, and only the ones that do something, so "what was the driver pressing" is a quick read
    // of the Events list instead of a bitmask.
    private final Map<String, BooleanSupplier> m_loggedButtons = Map.ofEntries(
        Map.entry("A", joystick.a()),
        Map.entry("B (failsafe shot)", joystick.b()),
        Map.entry("X (brake)", joystick.x()),
        Map.entry("Y (shoot + agitate)", joystick.y()),
        Map.entry("LB (intake)", joystick.leftBumper()),
        Map.entry("RB (unjam)", joystick.rightBumper()),
        Map.entry("LT (eject)", joystick.leftTrigger()),
        Map.entry("RT (just shoot)", joystick.rightTrigger()),
        Map.entry("Start (seed field pose)", joystick.start()),
        Map.entry("Back (shop test pose)", joystick.back()),
        Map.entry("POV up (re-home intake)", joystick.povUp())
    );

    /** Logs a named event the instant a mapped button is pressed or released. Call every loop.
     *  Lazy message (10/23 loop-overrun pass): building "Button: " + name 11 times a loop regardless of
     *  whether any button actually changed added up; now it's only built on an actual press/release. */
    public void logButtons() {
        m_loggedButtons.forEach((name, button) ->
            RobotLog.eventOnRise(name, button.getAsBoolean(), () -> "Button: " + name));
    }

    /** Rumbles the driver controller harder the closer the battery gets to browning out. Call every loop. */
    public void updateBrownoutRumble() {
        double target = 0.0;
        if (DriverStation.isEnabled()) {
            if (RobotController.isBrownedOut()) {
                target = 1.0;
            } else {
                double volts = RobotController.getBatteryVoltage();
                double brownoutVolts = RobotController.getBrownoutVoltage();
                target = MathUtil.clamp((kRumbleStartVolts - volts) / (kRumbleStartVolts - brownoutVolts), 0.0, 1.0);
            }
        }
        m_brownoutRumble = Math.max(target, m_brownoutRumble - kRumbleDecayPerLoop);
        joystick.getHID().setRumble(RumbleType.kBothRumble, m_brownoutRumble);
        SmartDashboard.putNumber("Brownout Rumble", m_brownoutRumble);
    }

    /** @return If the whole shooter is ready to shoot or not. */
    public boolean readyToShoot() {
        // The turret is NOT checked: it's always tracking the target, so a shot only waits for the shooter to be
        // at speed and the hood to be in place (the hood check also blocks shots near a trench).
        return m_readyDebouncer.calculate(
            shooter.isShooterReady(1.5) &&
            hood.atSetpoint());
    }

    private int m_shotCount = 0;

    /** One line per shot (each time the feeder starts): everything needed to see why a shot hit or missed and to
     *  tune the shooter/hood/pass tables. Also drops a pose in "Poses/Shot" so shots show up on the field. */
    private void logShot(String kind) {
        m_shotCount++;
        var state = drivetrain.getState();
        double speed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
        RobotLog.event(String.format(
            "SHOT #%d %s %s | dist %.2fm | shooter %.1f/%.1f RPS | hood %.2f/%.2f | turret err %.2f rot | speed %.2f m/s | pose (%.2f, %.2f, %.0f deg)",
            m_shotCount, kind, turret.isPassing() ? "PASS" : "HUB", turret.getShootingDistance(),
            shooter.getVelocityRPS(), shooter.getTargetRPS(), hood.getAngle(), hood.getSetpoint(),
            turret.getAimErrorMotorRotations(), speed,
            state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()));
        RobotLog.pose("Shot", state.Pose);
        SmartDashboard.putNumber("Shots/Count", m_shotCount);
    }

    /** Publishes, every loop, which part is holding up a shot and by how much. Shooter numbers only mean
     *  something while Y or RT is held (the shooter's target is 0 otherwise). */
    public void updateShotDiagnostics() {
        if (!RobotLog.telemetryTick()) {
            return;   // dashboard values at 10 Hz (loop time)
        }
        boolean shooterReady = shooter.isShooterReady(1.5);
        boolean turretReady = turret.canShootFromHere();   // info only, NOT required to shoot
        boolean hoodReady = hood.atSetpoint();
        SmartDashboard.putBoolean("Ready/0 ALL READY", shooterReady && hoodReady);

        // One line that says exactly what a shot is waiting on
        StringBuilder waiting = new StringBuilder();
        if (!shooterReady) waiting.append(String.format("SHOOTER %.0f of %.0f RPS. ", shooter.getVelocityRPS(), shooter.getTargetRPS()));
        if (!hoodReady) waiting.append(hood.isForcedDown() ? "HOOD held down near a trench. "
            : String.format("HOOD at %.2f, wants %.2f. ", hood.getAngle(), hood.getSetpoint()));
        SmartDashboard.putString("Feeder/Waiting on", waiting.length() == 0 ? "Nothing - ready to feed" : waiting.toString());

        // Events (logged once when they start, not every loop)
        RobotLog.eventOnRise("hoodForced", hood.isForcedDown(), "Hood held down near a trench");
        RobotLog.eventOnRise("turretRange", !turretReady, "Turret can't shoot from here: " + turret.getNotReadyReason());

        SmartDashboard.putBoolean("Ready/1 Shooter at speed", shooterReady);
        SmartDashboard.putNumber("Ready/1 Shooter target RPS", shooter.getTargetRPS());
        SmartDashboard.putNumber("Ready/1 Shooter actual RPS", shooter.getVelocityRPS());
        SmartDashboard.putNumber("Ready/1 Shooter error RPS", shooter.getTargetRPS() - shooter.getVelocityRPS());
        SmartDashboard.putNumber("Ready/1 Shooter allowed error RPS", shooter.getReadyTolerance(1.5));

        SmartDashboard.putBoolean("Ready/2 Turret (info only, not required)", turretReady);
        SmartDashboard.putString("Ready/2 Turret status", turret.getNotReadyReason());
        SmartDashboard.putNumber("Ready/2 Turret tracking error rots (info)", turret.getAimErrorMotorRotations());

        SmartDashboard.putBoolean("Ready/3 Hood in place", hoodReady);
        SmartDashboard.putString("Ready/3 Hood status", hood.isForcedDown() ? "Held down: near a trench" : "OK");
        SmartDashboard.putNumber("Ready/3 Hood error (need < 0.5)", hood.getAngle() - hood.getSetpoint());
    }

    /** SHOP TESTING (View button, never during a real match): tells the robot it is straight in front of our hub,
     *  2.6m from its center, with its back (turret) toward it. Without AprilTags the robot thinks it is at (0,0)
     *  facing the hub, where the back-facing turret can't reach, so it would never get ready to shoot. */
    public Command shopTestPose() {
        return drivetrain.runOnce(() -> {
            boolean red = onRedAlliance();
            drivetrain.resetPose(new Pose2d(
                red ? 16.54 - 2.0 : 2.0, 4.035,
                Rotation2d.fromDegrees(red ? 0 : 180)));
        }).ignoringDisable(true);
    }
}