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
    public final EvilIntake evilIntake = new EvilIntake(11, 12, upper);//spin ID should be set to 12
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
        m_passRpmMap.put(8.27, 45.0);
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
                // Check right trigger axis. If pulled more than 50%, set multiplier to 40% (0.4). Otherwise 100% (1.0).
                double slowModeMultiplier = joystick.getRightTriggerAxis() > 0.5 ? 0.4 : 1.0;

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
        joystick.y().whileTrue(fullShootCommand());
        joystick.b().whileTrue(failsafeShoot());
        joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        
        
       // joystick.leftBumper().whileTrue(new EvilIntakePiece(evilIntake, EvilIntakePosition.out))
       // .and(joystick.leftBumper().whileTrue(rollersystem.roll(100)));
        joystick.leftBumper().whileTrue(
            Commands.parallel(
                new EvilIntakePiece(evilIntake, EvilIntakePosition.out),
                rollersystem.runEnd(
                    () -> rollersystem.roll(100),
                    () -> rollersystem.rollerStop()
                )
            )
            );

        
        // Unjam: floor + lower + upper tunnel all spin backwards while held
        joystick.rightBumper().whileTrue(rollersystem.otherUnjam());
        // Eject: everything backwards like unjam, plus the intake goes out with its wheels reversed to spit fuel out
        joystick.leftTrigger().whileTrue(Commands.parallel(rollersystem.otherUnjam(), evilIntake.eject()));
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 
        // Shop testing only: pretend we're straight in front of the hub (disabled when connected to a real match)
        joystick.back().and(() -> !DriverStation.isFMSAttached()).onTrue(shopTestPose());
        
        // This will fire the shooter, move the hood, and slow the chassis
        joystick.rightTrigger().whileTrue(fullShootCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // EXPLANATION: The FMS calls this right before Auto starts to ask what sequence to run.
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /** This will coordinate all necessary subsystems and only shoot when they all report readiness */
    public Command fullShootCommand() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> calculateOptimalShooterRPS()),
            // Notice MoveTurret is gone! The default command we set above handles aiming.
            
            // Command C: Shoot only when the other subsystems are ready.
            new theYappy(rollersystem, () -> readyToShoot()),
            hood.hoodgo(() -> calculateOptimalHoodAngle())
        );
    }

    /** Auton-only shoot: never passes. Spins the shooter + hood for the hub the whole time (so it's
     *  already up to speed when the robot drives back into our zone) and only feeds while we're
     *  legally in our alliance zone and everything is ready. Works while driving (shoot on the move). */
    public Command autoShootCommand() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> hubShooterRPS(turret.getHubShootingDistance())),
            new theYappy(rollersystem, () -> !turret.isPassing() && readyToShoot()),
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
            new theYappy(rollersystem, () -> shooter.isShooterReady(2))
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
        boolean shooterReady = shooter.isShooterReady(1.5);
        boolean turretReady = turret.isTurretReady();
        boolean hoodReady = hood.atSetpoint();
        // Shows which part is holding up the shot
        SmartDashboard.putBoolean("Ready/1 Shooter at speed", shooterReady);
        SmartDashboard.putBoolean("Ready/2 Turret on target", turretReady);
        SmartDashboard.putBoolean("Ready/3 Hood in place", hoodReady);
        return m_readyDebouncer.calculate(shooterReady && turretReady && hoodReady);
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