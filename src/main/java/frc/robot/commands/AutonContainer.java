package frc.robot.commands;

import static frc.robot.Constants.PathPlannerConfigs.PP_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer{
    private RobotContainer robot;
    private CommandSwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robot) {
        this.robot = robot;
        this.drivetrain = robot.drivetrain;
        registerNamedCommands();

        // Attempt to load the pathplanner config from GUI
        // Fallback onto the config in Constants because it's better than crashing
        RobotConfig config = PP_CONFIG;
        try { config = RobotConfig.fromGUISettings(); }
        catch (Exception e) { e.printStackTrace(); }

        AutoBuilder.configure(
            drivetrain::getPose, 
            drivetrain::resetPose,
            drivetrain::getChassisSpeeds,
            (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> robot.onRedAlliance(),
            drivetrain
        );
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("DropIntake", robot.intake.rotateCommand(IntakePosition.out));
        NamedCommands.registerCommand("RaiseIntake", robot.intake.rotateCommand(IntakePosition.stow));
        NamedCommands.registerCommand("DriveIntake", new IntakePiece(robot.intake, IntakePosition.out));
        NamedCommands.registerCommand("Shoot", robot.fullShootCommand().withTimeout(5));
    }

    public SendableChooser<Command> buildAutonChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();
        chooser.setDefaultOption("Do Nothing", doNothing());
        chooser.addOption("Left Single", AutoBuilder.buildAuto("Left Trench Center"));
         chooser.addOption("Right Single", AutoBuilder.buildAuto("Right Trench Center"));
        chooser.addOption("Left Double", AutoBuilder.buildAuto("Left Trench Double"));
        chooser.addOption("Right Double", AutoBuilder.buildAuto("Right Trench Double"));
         chooser.addOption("Center Preload", AutoBuilder.buildAuto("Simple Center"));
          chooser.addOption("Left Preload", AutoBuilder.buildAuto("Trench Preload Left"));
           chooser.addOption("Right Preload", AutoBuilder.buildAuto("Trench Preload Right"));

        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return new WaitCommand(0);
    }
}