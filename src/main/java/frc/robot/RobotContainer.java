// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.MoveTurret;
import frc.robot.commands.SpindexYappy;
import frc.robot.commands.Zero;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindex;
import frc.robot.subsystems.Turret;
import frc.robot.wrappers.Limelight;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
   // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  //  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Limelight limelight = new Limelight("limelight");

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // --- NEW TURRET VARIABLES ---
    private AprilTagFieldLayout m_fieldLayout;
    public final Turret turret;

    final AutonContainer auton = new AutonContainer(this);
    final SendableChooser<Command> autonChooser = auton.buildAutonChooser();
    final Shooter shooter = new Shooter();
    final Spindex spindex = new Spindex();
    final Hood hood = new Hood();

    final Intake intaker = new Intake();


    public RobotContainer() {
       
               // NOTE: Make sure this is the right game year for whatever field you are testing on! 
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // --- NEW: INSTANTIATE TURRET ---
        // Passing in your "drivetrain" variable's Pose2d, and the layout we just loaded
        turret = new Turret(() -> drivetrain.getState().Pose, m_fieldLayout);
       
        SmartDashboard.putData("Auton Selector", autonChooser);
        configureBindings();
    
    }

    /** @return Whether the robot is on the red alliance or not */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.y().onTrue(new BumpHood(hood, 1));
        // joystick.a().onTrue(new BumpHood(hood, -1));
        // joystick.b().whileTrue(new BumpVelocity(shooter, spindex, 2)); 
        //joystick.x().onTrue(new Zero(shooter, hood, turret));
        joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        joystick.rightTrigger().whileTrue(fullShootCommand());


        joystick.leftBumper().whileTrue(new IntakePiece(intaker, IntakePosition.out));
        joystick.b().whileTrue(new IntakePiece(intaker, IntakePosition.zero));
        joystick.leftBumper().whileTrue(new IntakePiece(intaker, IntakePosition.out));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                // --- NEW: BIND TURRET AIM TO RIGHT BUMPER ---
        // While the driver holds the Right Bumper, the Turret will continuously calculate and track the hub
        //joystick.rightBumper().whileTrue(
        //    turret.run(() -> turret.alignToHub())
        //);
        // --------------------------------------------

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {

        return autonChooser.getSelected();
        // Simple drive forward auton
        //final var idle = new SwerveRequest.Idle();
        //return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
           // drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            
            /* 
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle) 
            */
        //);
    }

    /** This will coordinate all necessary subsystems and only shoot when they all report readiness */
    public Command fullShootCommand() {
        /* A parallel command group will run all of its subcommands
         * at the same time, and only end when all subcommands have finished.
         * Therefore, it is important to ensure that each subcommand has a working 
         * isFinished() function in it. If for some reason you can't get a command 
         * to end correctly, you can force it to last for exactly n seconds by 
         * changing the isFinished() to return false and adding a timeout like so:
         * new Shoot(shooter, 25).withTimeout(n)
         */
        /* To algorithmically decide how your robot should act, the preferred way
         * is to set up the command such that it takes a value as input (e.g. speed),
         * and then instead of passing a plain number as input, you simply type in your equation.
         * This also applies for using functions like those that come out of the limelight.
         * Example: Commanding the shooter using the squared height of a target
         * new Shoot(shooter, Math.pow(limelight.getTY(), 2))
         */
        return new ParallelCommandGroup(
            /* Command A: Rev the shooter */
            shooter.shoot(() -> calculateOptimalShooterRPS()),
            /* Command B: Move the hood */
            //new MoveHood(hood, 0),
            /* Command C: Aim the turret */
            new MoveTurret(turret),
            /* Command D: Shoot only when the other subsystems are ready */
            new SpindexYappy(spindex, () -> readyToShoot())
        );
    }

    public double calculateOptimalShooterRPS() {
        return turret.m_distanceToHubMeters * 2.692913 + 19.6;
    }

    public boolean readyToShoot() {
        return shooter.isShooterReady(2) &&
            turret.isTurretReady() &&
            hood.atSetpoint();
    }
}
