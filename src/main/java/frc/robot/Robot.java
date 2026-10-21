// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TorqueNados - FRC 5090

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
        RobotLog.start();   // DataLog + CTRE SignalLogger + command/event logs (USB stick in the RIO)
        // The CANivores come up a few seconds after the robot program starts. Every config sent before that fails
        // ("failed config with error TxFailed" for all swerve Talons/CANcoders in the 10/21 logs) and CTRE doesn't
        // retry, so the swerve ran without our current limits/offsets. Wait for both buses first.
        waitForCanBus("Jarvis", 8.0);
        waitForCanBus("Upper", 8.0);
        m_robotContainer = new RobotContainer();
        // Dashboard switch: false = ignore the Limelight completely (pure odometry). Shooting never needs it.
        SmartDashboard.putBoolean("Vision/Use camera", true);
        SmartDashboard.putString("Code Version", readCodeVersion());
    }

    /** Blocks (at startup only) until a CANivore bus is up, or the timeout passes */
    private static void waitForCanBus(String name, double timeoutSeconds) {
        if (RobotBase.isSimulation()) {
            return;   // no real CANivore in simulation
        }
        CANBus bus = new CANBus(name);
        double start = Timer.getFPGATimestamp();
        while (!bus.getStatus().Status.isOK() && Timer.getFPGATimestamp() - start < timeoutSeconds) {
            Timer.delay(0.1);
        }
        double waited = Timer.getFPGATimestamp() - start;
        if (bus.getStatus().Status.isOK()) {
            Timer.delay(0.5);   // give the devices on it a moment to finish booting too
            RobotLog.event(String.format("CAN bus %s ready after %.1f s", name, waited));
        } else {
            RobotLog.event("CAN bus " + name + " NOT READY after " + timeoutSeconds
                + " s: configs will fail (check the CANivore USB cable / power)");
        }
    }

    /** Branch + commit this code was built from (written by the writeVersionFile task in build.gradle) */
    private static String readCodeVersion() {
        try {
            return Files.readString(new File(Filesystem.getDeployDirectory(), "version.txt").toPath()).trim();
        } catch (IOException e) {
            DriverStation.reportWarning("Code Version unknown: " + e.getMessage(), false);
            return "unknown";
        }
    }

    @Override
    public void robotPeriodic() {
        boolean tel = RobotLog.telemetryTick();   // dashboard values at 10 Hz (loop time)

        double loopStart = Timer.getFPGATimestamp();
        RobotLog.tick();
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.updateBrownoutRumble();
        m_robotContainer.updateShotDiagnostics();
        RobotLog.eventOnRise("brownout", RobotController.isBrownedOut(), "BROWNOUT (battery " + RobotController.getBatteryVoltage() + " V)");
    /* This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation of how to use vision should be tuned per-robot and to the team's specification. */
    if (kUseLimelight) {
      updateVision("limelight");
      updateVision("limelight-left");

      var pose = m_robotContainer.drivetrain.getState().Pose;
      if (tel) RobotLog.pose("Robot", pose);
      if (tel) SmartDashboard.putNumber("111 drive pose X", pose.getX());
      if (tel) SmartDashboard.putNumber("111 drive pose Y",  pose.getY());
      if (tel) SmartDashboard.putNumber("111 drive pose heading", pose.getRotation().getDegrees());
      if (tel) SmartDashboard.putNumber("Vision/Seconds since last tag", Timer.getFPGATimestamp() - m_lastTagTime);
    }

        // How long OUR code took this loop (the budget is 20 ms). If this stays small while the Driver Station
        // still reports loop overruns, the time is going somewhere else (CAN error spam, garbage collection, CPU).
        double loopMs = (Timer.getFPGATimestamp() - loopStart) * 1000.0;
        if (tel) SmartDashboard.putNumber("Loop/Robot code ms", loopMs);
        RobotLog.eventOnRise("slowLoop", loopMs > 15.0, String.format("Slow robot code loop: %.1f ms", loopMs));
  }

    /** Feeds one Limelight into the pose estimator.
     *  Enabled: MegaTag2. It takes the heading from the Pigeon and only corrects X/Y, which keeps the
     *  pose steady for shoot-on-the-move (MegaTag1 single-tag headings jump around).
     *  Disabled: MegaTag1 with 2+ tags is allowed to fix the heading, so MegaTag2 starts the match
     *  with a correct heading even if the robot was placed crooked. */
    private void updateVision(String limelightName) {
      if (!SmartDashboard.getBoolean("Vision/Use camera", true)) {
        return;
      }
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
          recordVisionCorrection(driveState.Pose, mt1.pose, mt1.tagCount, mt1.rawFiducials);
          drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, VecBuilder.fill(0.5, 0.5, 0.5));
        }
      } else {
        var mt2 = Limelighthelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 != null && mt2.tagCount > 0) {
          recordVisionCorrection(driveState.Pose, mt2.pose, mt2.tagCount, mt2.rawFiducials);
          // Huge heading std dev = ignore vision heading, trust the Pigeon
          drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
        }
      }
    }

    private double m_lastTagTime = 0.0;

    /** How far the camera disagrees with odometry each time it sees tags. Small (under ~0.3m) = odometry is
     *  tracking well on its own. Big = odometry drifts when it can't see tags (wheel size, gearing, or gyro). */
    private void recordVisionCorrection(Pose2d odometryPose, Pose2d visionPose, int tagCount,
                                        Limelighthelpers.RawFiducial[] rawFiducials) {
      m_lastTagTime = Timer.getFPGATimestamp();
      if (!RobotLog.telemetryTick()) {
        return;   // dashboard values at 10 Hz (loop time)
      }
      if (rawFiducials != null) {
        // Which tags we localized from, how far away, and how unsure (ambiguity near 1 = unreliable)
        double[] ids = new double[rawFiducials.length];
        double[] dists = new double[rawFiducials.length];
        double worstAmbiguity = 0;
        for (int i = 0; i < rawFiducials.length; i++) {
          ids[i] = rawFiducials[i].id;
          dists[i] = rawFiducials[i].distToCamera;
          worstAmbiguity = Math.max(worstAmbiguity, rawFiducials[i].ambiguity);
        }
        SmartDashboard.putNumberArray("Vision/Tag IDs", ids);
        SmartDashboard.putNumberArray("Vision/Tag distances (m)", dists);
        SmartDashboard.putNumber("Vision/Worst tag ambiguity", worstAmbiguity);
      }
      RobotLog.pose("Vision", visionPose);   // compare against Poses/Robot to see odometry drift
      SmartDashboard.putNumber("Vision/Tags seen", tagCount);
      SmartDashboard.putNumber("Vision/Correction (m)", odometryPose.getTranslation().getDistance(visionPose.getTranslation()));
      SmartDashboard.putNumber("Vision/Heading disagreement (deg)", odometryPose.getRotation().minus(visionPose.getRotation()).getDegrees());
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
        // Test mode = hardware checks only: no commands/default commands run (nothing drives, turret stays put)
        CommandScheduler.getInstance().disable();
        m_robotContainer.evilIntake.startMotorTest();
    }

    @Override
    public void testPeriodic(){
        // Subsystem periodic() still runs (robotPeriodic) for the dashboard; this drives each intake motor alone
        m_robotContainer.evilIntake.runMotorTest();
        m_robotContainer.evilIntake.periodic();
    }

    @Override
    public void testExit(){
        m_robotContainer.evilIntake.endMotorTest();
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void simulationPeriodic(){}
}
