// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

      private final boolean kUseLimelight = true;

    // Log and replay timestamp and joystick data.
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    @Override
    public void robotInit(){
     /*  if (RobotController.getUserButton()) {
            m_robotContainer.intake.intakeCoast();
        } else {
            m_robotContainer.intake.intakeBrake();
        } */
    }
    
    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.updateBrownoutRumble();
    /* This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation of how to use vision should be tuned per-robot and to the team's specification. */
    if (kUseLimelight) {
      updateVision("limelight");
      updateVision("limelight-left");

      SmartDashboard.putNumber("111 drive pose X", m_robotContainer.drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("111 drive pose Y",  m_robotContainer.drivetrain.getState().Pose.getY());
    }
  }

    /** Feeds one Limelight into the pose estimator.
     *  Enabled: MegaTag2. It takes the heading from the Pigeon and only corrects X/Y, which keeps the
     *  pose steady for shoot-on-the-move (MegaTag1 single-tag headings jump around).
     *  Disabled: MegaTag1 with 2+ tags is allowed to fix the heading, so MegaTag2 starts the match
     *  with a correct heading even if the robot was placed crooked. */
    private void updateVision(String limelightName) {
      var drivetrain = m_robotContainer.drivetrain;
      var driveState = drivetrain.getState();

      // Vision is unreliable while spinning fast
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      if (Math.abs(omegaRps) >= 2.0) {
        return;
      }

      Limelighthelpers.SetRobotOrientation(limelightName, driveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

      if (DriverStation.isDisabled()) {
        var mt1 = Limelighthelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 != null && mt1.tagCount >= 2) {
          drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, VecBuilder.fill(0.5, 0.5, 0.5));
        }
      } else {
        var mt2 = Limelighthelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 != null && mt2.tagCount > 0) {
          // Huge heading std dev = ignore vision heading, trust the Pigeon
          drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
        }
      }
    }

    /* Used to be used for the intializing of the robot when disabled. No idea why it was commented out.
     * @Override
     * public void disabledInit(){} */

    @Override
    public void disabledPeriodic(){}

    @Override
    public void disabledExit(){}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void autonomousExit(){}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
            // autonCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic(){}

    @Override
    public void teleopExit(){}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic(){}

    @Override
    public void testExit(){}

    @Override
    public void simulationPeriodic(){}
}
