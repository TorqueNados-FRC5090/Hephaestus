// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
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
    public void robotInit(){}
    
    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    /* This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation of how to use vision should be tuned per-robot and to the team's specification. */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      
     
  
      Limelighthelpers.SetRobotOrientation("limelight",m_robotContainer.drivetrain.getgyroyaw().getDegrees(), 0, 0, 0, 0, 0);
      var llMeasurement = Limelighthelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        Limelighthelpers.SetRobotOrientation("limelight-left",m_robotContainer.drivetrain.getgyroyaw().getDegrees(), 0, 0, 0, 0, 0);
        var llMeasurementleft = Limelighthelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (llMeasurementleft != null && llMeasurementleft.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          m_robotContainer.drivetrain.addVisionMeasurement(llMeasurementleft.pose, llMeasurementleft.timestampSeconds);
        }
      
      SmartDashboard.putNumber("111 drive pose X", m_robotContainer.drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("111 drive pose Y",  m_robotContainer.drivetrain.getState().Pose.getY());
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
            // autonCommand.schedule();
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
