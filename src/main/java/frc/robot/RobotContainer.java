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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
    // --- EXTRA VARIABLES START ---
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
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
    private AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 

    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Spindex spindex = new Spindex();
    
    // SOTM UPDATE: Passing Pose, Speeds, and FieldLayout
    public final Turret turret = new Turret(
            () -> drivetrain.getState().Pose, 
            () -> drivetrain.getState().Speeds, 
            m_fieldLayout
        );

    final AutonContainer auton = new AutonContainer(this); 
    final SendableChooser<Command> autonChooser = auton.buildAutonChooser();
    // --- TURRET VARIABLES END ---

    // --- PASSING INTERPOLATION MAPS ---
    private final InterpolatingDoubleTreeMap m_passRpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passHoodMap = new InterpolatingDoubleTreeMap();

    // i have no idea what this does can someone who does explain it
    // EXPLANATION: This is the Constructor. It runs once when the robot boots up.
    public RobotContainer() {
        SmartDashboard.putData("Auton Selector", autonChooser);
        configureBindings();
        
        // Populate the passing maps with your field-length data
        m_passRpmMap.put(8.27, 45.0);
        m_passRpmMap.put(16.54, 60.0);
        m_passHoodMap.put(8.27, -1.5);
        m_passHoodMap.put(16.54, -1.5);
    }

    /** @return Whether the robot is on the red alliance or not. */
    public boolean onRedAlliance() { 
        return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    // another explanation of what this does as a whole would be nice!
    // EXPLANATION: This wires your physical Xbox controller to your robot's code commands.
    private void configureBindings() {
        drivetrain.setDefaultCommand(
         drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) 
          .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
         )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.b().whileTrue(failsafeShoot());
        joystick.x().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        joystick.rightTrigger().whileTrue(fullShootCommand());
        joystick.leftBumper().whileTrue(new IntakePiece(intake, IntakePosition.out));
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // plz explain!
    // EXPLANATION: The FMS calls this right before Auto starts to ask what sequence to run.
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /** This will coordinate all necessary subsystems and only shoot when they all report readiness */
    public Command fullShootCommand() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> calculateOptimalShooterRPS()),
            new MoveTurret(turret),
            new SpindexYappy(spindex, () -> readyToShoot()),
            hood.hoodgo(() -> calculateOptimalHoodAngle())
        );
    }

    /** Failsafe shoot that does not coordinate and instead sets everything to the minimum it can to shoot without an Apriltag. Should just shoot forward.  */
    public Command failsafeShoot() {
        return new ParallelCommandGroup(
            shooter.shoot(() -> 23), 
            turret.run(() -> turret.goToZero()),
            new SpindexYappy(spindex, () -> shooter.isShooterReady(2))
        );
    }

    // EXPLANATION: Calculates wheel speed based on SOTM distance.
    public double calculateOptimalShooterRPS() {
<<<<<<< HEAD
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();

        // 2. Override if Passing
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            return m_passRpmMap.get(targetDist);
        }

        // 3. New Equation 3/20/26 (For Hub Shooting)
        return (20.9 + 0.697 * targetDist + 0.243 * Math.pow(targetDist, 2));
=======
        double hubDist = turret.m_distanceToHubMeters;
        //return turret.m_distanceToHubMeters * 2.692913 + 18;
        /* Old equation used.
         return (turret.m_distanceToHubMeters * 135 + 1192)/60; */

        //New Equation 3/20/26 - New PIDs as well
        return (20.9 + 0.697 * hubDist + 0.243*Math.pow(hubDist, 2));
>>>>>>> 8457acf19dc0c35c1f9e500cac6bd85f9fa73fe0
    }

    // EXPLANATION: Calculates hood deflection based on SOTM distance.
    public double calculateOptimalHoodAngle() {
        // 1. Get Virtual SOTM Distance
        double targetDist = turret.getShootingDistance();
        double optimal = 0;
<<<<<<< HEAD

        // 2. Override if Passing
        if (SmartDashboard.getString("Turret/Mode", "SHOOTING").equals("PASSING")) {
            //optimal = m_passHoodMap.get(targetDist);
            optimal = -2.2;
        } 
        // 3. New Equation 3/20/26 (For Hub Shooting)
        else {
            if (targetDist >= 2.2) {
                optimal = (1 - (0.463 * targetDist));
            }
            else{
                optimal = 0;
            }
        }

=======
        /* Old Equation
        if(hubDist >= 1.74){optimal = -1 * (hubDist * 16.2 - 22.1 - 1.88 * Math.pow(hubDist, 2)) / 13.58086153;}
>>>>>>> 8457acf19dc0c35c1f9e500cac6bd85f9fa73fe0
        SmartDashboard.putNumber("Optimal Hood Angle", optimal);
        */

        //New Equation
        if(hubDist >= 2.2){optimal = 1 - 0.463*hubDist;}
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