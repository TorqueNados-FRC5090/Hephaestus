// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // Added for Passing
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.MoveTurret;
import frc.robot.commands.theYappy;
import frc.robot.commands.ok;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EvilIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.RollerSystem;
import frc.robot.subsystems.Turret;
import frc.robot.wrappers.Limelight;

public class RobotContainer {
    // --- EXTRA VARIABLES START ---
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    public final EvilIntake evilintake = new EvilIntake(11, 13);
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
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final RollerSystem rollersystem = new RollerSystem();
    
    // SOTM UPDATE: Passing Pose and Speeds (Removed FieldLayout)
    public final Turret turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> drivetrain.getState().Speeds
        );

    final AutonContainer auton = new AutonContainer(this); 
    final SendableChooser<Command> autonChooser = auton.buildAutonChooser();
    // --- TURRET VARIABLES END ---

    // --- PASSING INTERPOLATION MAPS ---
    private final InterpolatingDoubleTreeMap m_passRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passHoodMap = new InterpolatingDoubleTreeMap();

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
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    // EXPLANATION: This wires your physical Xbox controller to your robot's code commands.
    private void configureBindings() {
        drivetrain.setDefaultCommand(
         drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
          .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
         )

            );
            drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> {
    // Check right trigger axis. If pulled more than 50%, set multiplier to 40% (0.4). Otherwise 100% (1.0).
    double slowModeMultiplier = joystick.getRightTriggerAxis() > 0.5 ? 0.4 : 1.0;

    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowModeMultiplier)
    .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowModeMultiplier)
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowModeMultiplier);
    })
    );

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.b().whileTrue(failsafeShoot());
    joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

    // This will still fire the shooter, but now the default drive command above will simultaneously slow the chassis
    joystick.rightTrigger().whileTrue(fullShootCommand());


        joystick.a().whileTrue(new ok(evilintake));
        joystick.b().whileTrue(failsafeShoot());
        joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        joystick.rightTrigger().whileTrue(fullShootCommand());
        joystick.leftBumper().whileTrue(new IntakePiece(intake, IntakePosition.out));
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 
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
            new MoveTurret(turret),
            // Command C: Shoot only when the other subsystems are ready.
            new theYappy(rollersystem, () -> readyToShoot()),
            hood.hoodgo(() -> calculateOptimalHoodAngle())
        );
        /* --- UNUSED LINES START ---
         * // This was between Command A and Command B as the old Command B.
         * // Command B: Move the hood.
         * new MoveHood(hood, 0),
         * new MoveHood(hood, calculateOptimalHoodAngle()),
        --- UNUSED LINES END --- */
    }

    /** Failsafe shoot that does not coordinate and instead sets everything to the minimum it can to shoot without an Apriltag. Should just shoot forward.  */
    public Command failsafeShoot() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> 23), 
            turret.run(() -> turret.goToZero()),
            new theYappy(rollersystem, () -> shooter.isShooterReady(2))
        );
    }

    // EXPLANATION: Calculates wheel speed based on SOTM distance.
    public double calculateOptimalShooterRPS() {
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();

        // 2. Override if Passing
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            return m_passRpmMap.get(targetDist);
        }
        
        // 3. New Equation 3/20/26 (For Hub Shooting)
        return (20.9 + 0.697 * targetDist + 0.243 * Math.pow(targetDist, 2));
    }

    // EXPLANATION: Calculates hood deflection based on SOTM distance.
    public double calculateOptimalHoodAngle() {
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();
        double optimal = 0;

        // 2. Override if Passing
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            optimal = m_passHoodMap.get(targetDist);
        } 
        // 3. New Equation 3/20/26 (For Hub Shooting)
        else {
            if (targetDist >= 2.2) {
                optimal = (1 - (0.463 * targetDist));
            } else {
                optimal = 0;
            }
        }

        SmartDashboard.putNumber("Optimal Hood Angle", optimal);
        //New Equation
        if(targetDist >= 2.2){optimal = (1 - 0.463*targetDist);}
        SmartDashboard.putNumber("Optimal Hood Angle", optimal);

        return optimal;
    }

    /** @return If the whole shooter is ready to shoot or not. */
    public boolean readyToShoot() {
        return shooter.isShooterReady(1.5) &&
            turret.isTurretReady() &&
            hood.atSetpoint();
    }
}