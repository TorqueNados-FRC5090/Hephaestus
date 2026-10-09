package frc.robot.commands;

import static frc.robot.Constants.PathPlannerConfigs.PP_CONFIG;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** A container that stores various procedures for the autonomous portion of the game */
public class AutonContainer{
    private RobotContainer robotContainer;
    private CommandSwerveDrivetrain drivetrain;

    /** Constructs an AutonContainer object */ 
    public AutonContainer(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
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
            () -> robotContainer.onRedAlliance(),
            drivetrain
        );
    }

    private void registerNamedCommands() {
        // DropIntake/RaiseIntake finish instantly (they used to run forever and froze any auto that used them in a row).
        // The pivot holds its position after they end.
        NamedCommands.registerCommand("DropIntake", robotContainer.evilIntake.runOnce(() -> robotContainer.evilIntake.evilyummy(EvilIntakePosition.out)));
        NamedCommands.registerCommand("RaiseIntake", robotContainer.evilIntake.runOnce(() -> {
            robotContainer.evilIntake.evilyummy(EvilIntakePosition.in);
            robotContainer.evilIntake.evileryummy(0);
        }));
        // Never ends on its own: only use it inside a deadline group with a path as the deadline
        NamedCommands.registerCommand("DriveIntake", new EvilIntakePiece(robotContainer.evilIntake, EvilIntakePosition.out));
        // Stand-still shoot. Hub only, never passes.
        NamedCommands.registerCommand("Shoot", robotContainer.autoShootCommand().withTimeout(5));
        // Short stand-still shoot to empty whatever is left after shooting on the move
        NamedCommands.registerCommand("ShootFinish", robotContainer.autoShootCommand().withTimeout(1.5));
        // Never ends on its own: put it in a deadline group with a path to shoot while driving
        NamedCommands.registerCommand("ShootOnMove", robotContainer.autoShootCommand());
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
        chooser.addOption("Left Double Swipe (SOTM)", AutoBuilder.buildAuto("Left Double Swipe"));
        chooser.addOption("Right Double Swipe (SOTM)", AutoBuilder.buildAuto("Right Double Swipe"));

        return chooser;
    }

    /** Auton that does nothing */
    public Command doNothing() {
        return new WaitCommand(0);
    }
}