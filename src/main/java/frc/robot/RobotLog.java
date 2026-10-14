package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Season-independent logging. Reuse this file every year.
 *
 * <ul>
 *   <li>WPILib DataLog: every NetworkTables value (all dashboard values) + joysticks, buttons, enable state,
 *       match info, to a .wpilog on the USB stick in the roboRIO (falls back to the RIO if none).</li>
 *   <li>CTRE SignalLogger: every CTRE device (current, voltage, velocity, temps, faults) to .hoot files.</li>
 *   <li>Command start / end / interrupt, as text in "Commands".</li>
 *   <li>Short text events (rising edges only) in "Events".</li>
 *   <li>Poses as structs so AdvantageScope draws them on the field.</li>
 * </ul>
 * Open everything in AdvantageScope (only as a viewer, no AdvantageKit needed).
 */
public final class RobotLog {
    private RobotLog() {}

    private static StringLogEntry s_commands;
    private static StringLogEntry s_events;
    private static final Map<String, StructPublisher<Pose2d>> s_poses = new HashMap<>();
    private static final Map<String, StructArrayPublisher<Pose2d>> s_poseArrays = new HashMap<>();
    private static final Map<String, Boolean> s_lastState = new HashMap<>();

    /** Call once, first thing in Robot() */
    public static void start() {
        DataLogManager.start();   // picks the USB stick (/u) automatically when one is plugged in
        DataLogManager.logNetworkTables(true);
        // Also save the console (Driver Station messages), including the loop-overrun printouts that name the
        // slow part, and any CAN/Phoenix error spam
        DataLogManager.logConsoleOutput(true);
        DriverStation.startDataLog(DataLogManager.getLog());

        // CTRE logs: USB stick if present, otherwise CTRE's default location on the RIO
        if (new File("/u").canWrite()) {
            SignalLogger.setPath("/u/ctre-logs/");
        }
        SignalLogger.start();

        s_commands = new StringLogEntry(DataLogManager.getLog(), "Commands");
        s_events = new StringLogEntry(DataLogManager.getLog(), "Events");

        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(cmd -> s_commands.append("START " + cmd.getName()));
        scheduler.onCommandFinish(cmd -> s_commands.append("END " + cmd.getName()));
        scheduler.onCommandInterrupt((cmd, interruptor) -> s_commands.append("INTERRUPTED " + cmd.getName()
            + interruptor.map(by -> " by " + by.getName()).orElse("")));
    }

    /** A one-off text event, e.g. "Brownout" or "Intake homed" */
    public static void event(String message) {
        if (s_events != null) {
            s_events.append(message);
        }
        DataLogManager.log(message);
    }

    /** Logs an event only when the condition turns true (not every loop it stays true) */
    public static void eventOnRise(String key, boolean condition, String message) {
        boolean last = s_lastState.getOrDefault(key, false);
        if (condition && !last) {
            event(message);
        }
        s_lastState.put(key, condition);
    }

    /** Publishes a pose that AdvantageScope can draw on the field (also recorded in the log) */
    public static void pose(String name, Pose2d pose) {
        s_poses.computeIfAbsent(name, n ->
            NetworkTableInstance.getDefault().getStructTopic("Poses/" + n, Pose2d.struct).publish()).set(pose);
    }

    /** Publishes a list of poses (e.g. the active auto path) */
    public static void poses(String name, List<Pose2d> poses) {
        s_poseArrays.computeIfAbsent(name, n ->
            NetworkTableInstance.getDefault().getStructArrayTopic("Poses/" + n, Pose2d.struct).publish())
            .set(poses.toArray(new Pose2d[0]));
    }
}
