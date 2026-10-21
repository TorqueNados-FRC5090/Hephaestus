package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;
import frc.robot.RobotLog;

/**
 * Rack-and-pinion intake: two NEO Vortex motors (one per side, Talon FXS controllers) drive the racks,
 * plus the intake wheels (Talon FX).
 *
 * <p>How it works:
 * <ul>
 *   <li>Motor 11 (leader) is commanded; motor 14 follows it OPPOSED (the motors face each other). Confirmed on the
 *       robot 10/21: with Aligned, 11 drove out while 14 drove in and the intake bound. Positive = intake goes OUT
 *       (confirmed: +1 V on 11 moved it out).</li>
 *   <li>Every move uses Motion Magic, so speed and acceleration are always limited (unlimited, this gearing
 *       would slam the rack at ~200 in/s). Standard Motion Magic on purpose: it works without a Phoenix Pro
 *       license (Pro-only controls make an unlicensed Talon refuse to move).</li>
 *   <li>Zero: at power-on the intake is assumed fully in (0"). In a real match that is always true (the robot
 *       starts inside its frame perimeter), so on the field it never homes by itself and auto is unaffected.
 *       Off the field (shop/practice, no FMS) it homes on the first enable, because the robot may have been
 *       powered on with the intake out. Homing = slowly push out against the 16" extension stop, detect the
 *       stall, call that 16". D-pad up / dashboard Re-home homes on request any time (e.g. after a big hit).
 *       If homing fails it falls back to the power-on zero and keeps working.</li>
 *   <li>A low stator current limit makes the rack act like a force-capped spring: a hit pushes it in instead of
 *       skipping the chain, then Motion Magic brings it back.</li>
 *   <li>Sync guard: if the two sides drift more than 1" apart (one side blocked), both stop until re-homed.</li>
 * </ul>
 * Commands only set a target; periodic() is the one place that talks to the rack motors.
 */
public class EvilIntake extends SubsystemBase {
     // ================= GEARING / TRAVEL =================
     /** Vortex 14T sprocket -> 24T sprocket (chain, 1.71:1), then a 10T 10DP pinion (1.0" pitch diameter) on the rack.
      *  1 inch of rack travel = (24/14) / (pi * 1.0") = 0.546 motor rotations. */
     public static final double kMotorRotationsPerInch = (24.0 / 14.0) / (Math.PI * 1.0);
     /** Extension hard stop (inches). Auto-zero pushes against this and sets it as the reference. */
     private static final double kExtendStopInches = 16.0;
     /** Never command closer than this to the extension stop (Motion Magic would push into it) */
     private static final double kMaxCommandInches = 15.5;
     /** Never command into the retract stop (fuel gets squeezed / jams) */
     private static final double kMinCommandInches = 0.2;

     // ================= SPEED (Motion Magic) =================
     // Change these live from the dashboard (Intake/Tune) and copy the good values back here.
     private static final double kDefaultCruiseInPerSec = 40.0;   // full 14" stroke in ~0.5 s
     private static final double kDefaultAccelInPerSec2 = 150.0;
     // Agitate follows a slow back-and-forth (kAgitatePeriodSeconds), always slower than the cruise speed.

     // ================= AUTO ZERO (hard-stop homing) =================
     /** Volts pushing OUT while homing. Starts at 0.8 V (~14 in/s) and steps up if the rack doesn't move. */
     private static final double kHomingStartVolts = 0.8;
     private static final double kHomingMaxVolts = 2.5;
     /** Volts added per second while the rack still hasn't moved */
     private static final double kHomingRampVoltsPerSec = 1.0;
     /** The rack must travel at least this far before a stall counts as "hit the stop" (otherwise "it never
      *  moved" would look exactly like "it hit the stop") */
     private static final double kHomingMinTravelInches = 2.0;
     /** Below this speed (motor rot/s) the rack counts as stopped */
     private static final double kHomingStallRps = 0.5;
     /** Must stay stopped this long to count as "hit the stop" */
     private static final double kHomingStallSeconds = 0.20;
     /** Ignore the first part of homing while the motor gets moving */
     private static final double kHomingIgnoreSeconds = 0.30;
     /** Give up after this long (something is jammed or a motor is unplugged) */
     private static final double kHomingTimeoutSeconds = 4.0;

     // ================= SYNC GUARD =================
     /** If the two sides disagree by more than this (inches) for kSyncFaultSeconds, both stop until re-homed */
     private static final double kSyncFaultInches = 1.0;
     private static final double kSyncFaultSeconds = 0.25;
     /** false = only warn (dashboard + log). Set true once "Intake/Sides differ (in)" has been seen to stay near
      *  0 during normal moves (proves the follower reading is the right way round). */
     private static final boolean kSyncGuardStopsIntake = false;

     // ================= CURRENT LIMITS (why) =================
     /** Rack motors, stator: 40 A ~= 21 lb of push per side (~42 lb with both), about what the old Kraken intake had
      *  (~43 lb). It was 20 A (~10 lb/side) as a soft "spring", but in the 10/21 logs the rack stalled at the limit
      *  and never moved. Once the intake works reliably, lower this step by step to get the give-on-impact back. */
     private static final double kRackStatorLimitAmps = 40.0;
     /** Rack motors, supply: caps battery draw per side so the intake can't add much to a brownout */
     private static final double kRackSupplyLimitAmps = 30.0;
     /** Intake wheels, stator: enough grip to pull fuel in; limits heat if the wheels stall on a jam */
     private static final double kWheelStatorLimitAmps = 40.0;
     /** Intake wheels, supply: battery protection */
     private static final double kWheelSupplyLimitAmps = 30.0;

     // ================= GAINS (starting values, tune on the dashboard) =================
     // Units: volts per motor rotation of error (kP), volts per rot/s (kV), volts (kS).
     private static final double kDefaultP = 3.0;   // pushes harder the further it is from target
     private static final double kDefaultI = 0.0;   // normally not needed
     private static final double kDefaultD = 0.0;   // add a little if it overshoots or bounces
     private static final double kDefaultS = 0.25;  // volts to get past friction
     private static final double kDefaultV = 0.11;  // ~12 V / Vortex free speed (113 rot/s)
     private static final double kDefaultA = 0.0;

     // ================= WHEELS / AGITATE =================
     /** Roller power while intaking. Negative = pulls fuel in (same as EvilIntakePiece). */
     private static final double kRollerIntakeSpeed = -1.0;
     /** Roller power while ejecting (spits fuel back out the intake) */
     private static final double kRollerEjectSpeed = 1.0;
     /** How far in the rack slides during agitate (inches). Only part way: all the way in with fuel jams it. */
     private static final double kAgitateInInches = 8.0;
     /** Seconds for one full out -> in -> out cycle. Bigger = slower */
     private static final double kAgitatePeriodSeconds = 2.0;

     // ================= FOLLOWER DIRECTION =================
     /** Motor 14 turns opposite to motor 11 (they face each other). Used everywhere the follower is set up. */
     private static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed;
     /** Because it's opposed, motor 14's own position reads the opposite sign of motor 11's */
     private static final double kFollowerPositionSign = -1.0;

     // ================= HARDWARE =================
     final TalonFXS intakeMotor;      // leader (CAN 11)
     final TalonFXS intakeMotorTwo;   // follower (CAN 14), opposed
     final TalonFX spinMotor;         // intake wheels (CAN 12)

     // FOC OFF on the rack Talon FXSs: FOC needs a Phoenix Pro license on these devices, and in the 10/21 log they
     // reported an unlicensed fault and put out 0 V. Normal commutation works with or without the license.
     private final MotionMagicVoltage m_moveRequest = new MotionMagicVoltage(0).withEnableFOC(false);
     private final VoltageOut m_homingRequest = new VoltageOut(0).withEnableFOC(false);
     private final NeutralOut m_stopRequest = new NeutralOut();
     final DutyCycleOut spinRequest = new DutyCycleOut(0).withEnableFOC(true);

     // ================= STATE =================
     private enum State { NOT_HOMED, HOMING, READY, SIDES_OUT_OF_SYNC }
     private State m_state = State.NOT_HOMED;
     /** true = home even when connected to the field (driver/dashboard asked for it) */
     private boolean m_homeRequested = false;
     /** Where the current zero came from (dashboard) */
     private String m_zeroSource = "power-on (intake assumed fully in)";
     private final Timer m_homingTimer = new Timer();
     private final Timer m_stallTimer = new Timer();
     private final Timer m_syncTimer = new Timer();
     private double m_targetInches = EvilIntakePosition.in.getInches();
     private double m_homingStartInches = 0;
     private boolean m_homingMoved = false;
     private double m_homingVolts = kHomingStartVolts;
     private boolean m_motorTest = false;
     private int m_intakeId;

    /** Constructs the intake
     * @param intakeID CAN ID of the leader rack motor (Talon FXS + Vortex), positive = out
     * @param intakeTwoID CAN ID of the follower rack motor (Talon FXS + Vortex), mounted opposite
     * @param spinID CAN ID of the intake wheels (Talon FX)
     */
    public EvilIntake(int intakeID, int intakeTwoID, int spinID, CANBus canbus){
        m_intakeId = intakeID;
        intakeMotor = new TalonFXS(intakeID, canbus);
        intakeMotorTwo = new TalonFXS(intakeTwoID, canbus);
        spinMotor = new TalonFX(spinID, canbus);

        // --- RACK MOTORS (Talon FXS + NEO Vortex) ---
        TalonFXSConfiguration rackConfig = new TalonFXSConfiguration();
        // Tells the Talon FXS a Vortex is attached (through the REV Vortex Solo Adapter). Nothing runs without it.
        rackConfig.Commutation.MotorArrangement = MotorArrangementValue.VORTEX_JST;
        // AdvancedHallSupport (better Vortex commutation) is a Phoenix Pro feature: left OFF so the intake runs even
        // if these Talons aren't covered by the license. Turn on once "Intake/Faults" never shows UNLICENSED.
        rackConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Disabled;
        // Leader: positive = OUT. If the intake goes IN when it should go out, flip this to Clockwise_Positive.
        rackConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Coast: the rack can give when hit while the robot is disabled
        rackConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // No limit switches are wired. The Talon FXS limit inputs share the data port the Vortex adapter plugs into,
        // so leave them off; otherwise a floating input can read "pressed" and block motion in that direction.
        rackConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        rackConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        rackConfig.CurrentLimits.StatorCurrentLimit = kRackStatorLimitAmps;
        rackConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rackConfig.CurrentLimits.SupplyCurrentLimit = kRackSupplyLimitAmps;
        rackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rackConfig.Slot0 = gainsFromDashboardDefaults();
        rackConfig.MotionMagic = motionMagicFor(kDefaultCruiseInPerSec, kDefaultAccelInPerSec2);
        // Push toward the target to get past friction (kS sign follows the error)
        rackConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        intakeMotor.getConfigurator().apply(rackConfig);
        intakeMotorTwo.getConfigurator().apply(rackConfig);   // same limits on the follower (followers don't copy them)
        // Motor 14 copies motor 11's output, reversed (they face each other)
        intakeMotorTwo.setControl(new Follower(intakeID, kFollowerAlignment));
        // Old faults (e.g. the unlicensed fault from earlier code) would otherwise show on the dashboard forever
        intakeMotor.clearStickyFaults();
        intakeMotorTwo.clearStickyFaults();

        // At power-on assume the intake is fully in. Auto zero corrects it on the first enable.
        intakeMotor.setPosition(0);
        intakeMotorTwo.setPosition(0);

        // --- INTAKE WHEELS ---
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinConfig.CurrentLimits.StatorCurrentLimit = kWheelStatorLimitAmps;
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spinConfig.CurrentLimits.SupplyCurrentLimit = kWheelSupplyLimitAmps;
        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spinMotor.getConfigurator().apply(spinConfig);

        publishTuningControls();
    }

    // =====================================================================
    // Commands / API used by the rest of the code (names kept the same)
    // =====================================================================

    /** Move the rack to a named position (normal speed) */
    public void evilyummy(EvilIntakePosition pos){
        setTargetInches(pos.getInches());
    }

    /** Old placeholder, does nothing */
    public void evilyummy(){
    }

    /** Intake wheel power, -1..1 (negative pulls fuel in) */
    public void evileryummy(double speed){
        spinMotor.setControl(spinRequest.withOutput(speed));
    }

    public Command evilestyummy(EvilIntakePosition pos){
        return run(() -> evilyummy(pos));
    }

    /** Rollers on while the rack slowly slides between pickup and kAgitateInInches, squeezing the hopper so
     *  fuel keeps flowing into the feeder while we shoot. Leaves the rack out and rollers off when it ends. */
    public Command agitate() {
        return agitate(EvilIntakePosition.out);
    }

    /** Same as agitate(), but leaves the rack at endPosition when it stops (teleop puts it back in) */
    public Command agitate(EvilIntakePosition endPosition) {
        Timer timer = new Timer();
        double out = EvilIntakePosition.out.getInches();
        return startRun(
            timer::restart,
            () -> {
                // 0 -> 1 -> 0 over one period, starting from out
                double inAmount = (1 - Math.cos(2 * Math.PI * timer.get() / kAgitatePeriodSeconds)) / 2;
                setTargetInches(out + (kAgitateInInches - out) * inAmount);
                spinMotor.setControl(spinRequest.withOutput(kRollerIntakeSpeed));
            })
            .finallyDo(() -> {
                evilyummy(endPosition);
                spinMotor.setControl(spinRequest.withOutput(0));
            });
    }

    /** Intake out with the wheels spinning backwards to spit fuel out. Pulls back in and stops when released,
     *  same as the intake button. */
    public Command eject() {
        return run(() -> {
            evilyummy(EvilIntakePosition.out);
            spinMotor.setControl(spinRequest.withOutput(kRollerEjectSpeed));
        }).finallyDo(() -> {
            evilyummy(EvilIntakePosition.in);
            spinMotor.setControl(spinRequest.withOutput(0));
        });
    }

    /** Re-run the auto zero (e.g. after a big hit, or after the sides got out of sync). Runs when enabled. */
    public Command requestHomeCommand() {
        return new InstantCommand(this::requestHome).ignoringDisable(true);
    }

    public void requestHome() {
        m_state = State.NOT_HOMED;
        m_homeRequested = true;
    }

    public boolean isHomed() {
        return m_state == State.READY;
    }

    /** @return leader position in motor rotations */
    public double getAngle(){
        return intakeMotor.getPosition().getValueAsDouble();
    }

    /** @return leader position in inches of rack travel */
    public double getInches(){
        return getAngle() / kMotorRotationsPerInch;
    }

    private void setTargetInches(double inches) {
        m_targetInches = Math.max(kMinCommandInches, Math.min(kMaxCommandInches, inches));
    }

    // =====================================================================
    // The one place that drives the rack motors
    // =====================================================================
    @Override
    public void periodic() {
        if (m_motorTest) {
            publishTelemetry(getInches(), kFollowerPositionSign * intakeMotorTwo.getPosition().getValueAsDouble() / kMotorRotationsPerInch,
                intakeMotor.getVelocity().getValueAsDouble());
            return;   // Test mode: runMotorTest() is driving the motors, don't touch them here
        }
        double leaderInches = getInches();
        // Opposed follower: flip its reading so both are "inches out" 
        double followerInches = kFollowerPositionSign * intakeMotorTwo.getPosition().getValueAsDouble() / kMotorRotationsPerInch;
        double velocityRps = intakeMotor.getVelocity().getValueAsDouble();

        if (DriverStation.isDisabled() && m_state == State.HOMING) {
            m_state = State.NOT_HOMED;   // disabled mid-homing: start over on the next enable
        }

        if (DriverStation.isEnabled()) {
            switch (m_state) {
                case NOT_HOMED -> {
                    if (DriverStation.isFMSAttached() && !m_homeRequested) {
                        // Real match: the robot started inside its frame, so the intake was at the 0" stop at
                        // power-on and that zero is correct. Don't spend auto time homing.
                        m_state = State.READY;
                        m_syncTimer.restart();
                        RobotLog.event("Intake: match start, using the power-on zero (no homing)");
                    } else {
                        startHoming();
                    }
                }
                case HOMING -> runHoming(velocityRps);
                case READY -> {
                    checkSync(leaderInches, followerInches);
                    if (m_state == State.READY) {
                        driveToTarget();
                    }
                }
                // Out of sync: hold still until someone re-homes (D-pad up or dashboard)
                case SIDES_OUT_OF_SYNC -> intakeMotor.setControl(m_stopRequest);
            }
        }

        publishTelemetry(leaderInches, followerInches, velocityRps);
    }

    private void publishTelemetry(double leaderInches, double followerInches, double velocityRps) {
        if (!RobotLog.telemetryTick()) {
            return;   // dashboard values at 10 Hz (loop time)
        }
        SmartDashboard.putString("Intake/Status", m_motorTest ? "MOTOR TEST" : m_state.toString());
        SmartDashboard.putBoolean("Intake/Homed", isHomed());
        SmartDashboard.putString("Intake/Zero from", m_zeroSource);
        SmartDashboard.putNumber("Intake/Position (in)", leaderInches);
        SmartDashboard.putNumber("Intake/Target (in)", m_targetInches);
        SmartDashboard.putNumber("Intake/Error (in)", m_targetInches - leaderInches);
        SmartDashboard.putNumber("Intake/Velocity (in per s)", velocityRps / kMotorRotationsPerInch);
        // Should stay near 0. If it grows steadily on every move, the follower's direction/sign is wrong.
        SmartDashboard.putNumber("Intake/Sides differ (in)", leaderInches - followerInches);
        SmartDashboard.putNumber("Intake/Leader current (A)", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Follower current (A)", intakeMotorTwo.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Leader volts", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller voltage", spinMotor.getMotorVoltage().getValueAsDouble());
        // Anything stopping a rack Talon FXS from driving, right now (not sticky)
        String faults = activeFaults("leader", intakeMotor) + activeFaults("follower", intakeMotorTwo);
        SmartDashboard.putString("Intake/Faults", faults.isEmpty() ? "none" : faults);
    }

    private static String activeFaults(String name, TalonFXS talon) {
        StringBuilder f = new StringBuilder();
        if (talon.getFault_UnlicensedFeatureInUse().getValue()) f.append(name).append(": UNLICENSED feature. ");
        if (talon.getFault_ForwardHardLimit().getValue()) f.append(name).append(": forward limit switch. ");
        if (talon.getFault_ReverseHardLimit().getValue()) f.append(name).append(": reverse limit switch. ");
        if (talon.getFault_BridgeBrownout().getValue()) f.append(name).append(": bridge brownout. ");
        if (talon.getFault_Undervoltage().getValue()) f.append(name).append(": low voltage. ");
        // Hall-sensor faults: the Talon FXS turns its output OFF. In the 10/21 log motor 11 had these (plus
        // impossible +/-500 rot/s speed readings) = bad sensor connection between the Vortex adapter and the Talon.
        if (talon.getFault_HallSensorMissing().getValue()) f.append(name).append(": HALL SENSOR MISSING (check sensor cable). ");
        if (talon.getFault_DriveDisabledHallSensor().getValue()) f.append(name).append(": DRIVE DISABLED - hall sensor (check sensor cable). ");
        if (talon.getFault_MotorArrangementNotSelected().getValue()) f.append(name).append(": motor type not set. ");
        if (talon.getFault_BridgeShort().getValue()) f.append(name).append(": BRIDGE SHORT (motor wiring). ");
        if (talon.getFault_5V().getValue()) f.append(name).append(": 5V sensor supply fault. ");
        return f.toString();
    }

    private void driveToTarget() {
        intakeMotor.setControl(m_moveRequest.withPosition(m_targetInches * kMotorRotationsPerInch));
    }

    /** Motion Magic speed limits in motor units from inches */
    private static MotionMagicConfigs motionMagicFor(double cruiseInPerSec, double accelInPerSec2) {
        return new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(cruiseInPerSec * kMotorRotationsPerInch)
            .withMotionMagicAcceleration(accelInPerSec2 * kMotorRotationsPerInch)
            .withMotionMagicJerk(accelInPerSec2 * kMotorRotationsPerInch * 10);   // ~0.1 s to full accel
    }

    private void startHoming() {
        m_state = State.HOMING;
        m_homingTimer.restart();
        m_stallTimer.restart();
        m_homingStartInches = getInches();
        m_homingMoved = false;
        m_homingVolts = kHomingStartVolts;
        RobotLog.event("Intake homing: pushing out to the extension stop");
    }

    private void runHoming(double velocityRps) {
        if (!m_homingMoved) {
            if (getInches() - m_homingStartInches >= kHomingMinTravelInches) {
                m_homingMoved = true;   // it's travelling out; keep this voltage until it reaches the stop
            } else if (m_homingTimer.get() > kHomingIgnoreSeconds) {
                // Not moving yet (friction / chain): push a little harder, up to kHomingMaxVolts
                m_homingVolts = Math.min(kHomingMaxVolts, m_homingVolts + kHomingRampVoltsPerSec * 0.02);
            }
        }
        intakeMotor.setControl(m_homingRequest.withOutput(m_homingVolts));
        SmartDashboard.putNumber("Intake/Homing volts", m_homingVolts);

        boolean moving = Math.abs(velocityRps) > kHomingStallRps;
        if (moving || !m_homingMoved || m_homingTimer.get() < kHomingIgnoreSeconds) {
            m_stallTimer.restart();
        }
        if (m_stallTimer.get() >= kHomingStallSeconds) {
            // Stopped against the extension stop: that is 16"
            double stopRotations = kExtendStopInches * kMotorRotationsPerInch;
            intakeMotor.setPosition(stopRotations);
            intakeMotorTwo.setPosition(kFollowerPositionSign * stopRotations);
            m_syncTimer.restart();
            m_state = State.READY;
            m_homeRequested = false;
            m_zeroSource = "hard stop (homed)";
            RobotLog.event(String.format("Intake homed in %.2f s", m_homingTimer.get()));
        } else if (m_homingTimer.get() > kHomingTimeoutSeconds) {
            // Don't lock the intake up for the rest of the match: keep the zero we had and carry on
            m_syncTimer.restart();
            m_state = State.READY;
            m_homeRequested = false;
            String why = m_homingMoved
                ? "moved but never stopped at the 16\" stop (chain skipping? stop missing?)"
                : "never moved even at " + kHomingMaxVolts + " V (wrong direction = pushing into the IN stop? "
                  + "motor/adapter not plugged in? license fault? chain off?)";
            m_zeroSource = "power-on (HOMING FAILED: " + why + ")";
            RobotLog.event("Intake homing FAILED: " + why + ". Kept the power-on zero; fix it, then re-home");
        }
    }

    private void checkSync(double leaderInches, double followerInches) {
        if (Math.abs(leaderInches - followerInches) <= kSyncFaultInches) {
            m_syncTimer.restart();
        } else if (m_syncTimer.get() >= kSyncFaultSeconds) {
            RobotLog.eventOnRise("intakeSync", true, String.format(
                "Intake sides OUT OF SYNC (%.2f in vs %.2f in)%s", leaderInches, followerInches,
                kSyncGuardStopsIntake ? ": stopped, re-home to clear" : " (warning only, guard not armed)"));
            if (kSyncGuardStopsIntake) {
                m_state = State.SIDES_OUT_OF_SYNC;
                intakeMotor.setControl(m_stopRequest);
            }
            return;
        }
        RobotLog.eventOnRise("intakeSync", false, "");
    }

    // =====================================================================
    // MOTOR TEST (Driver Station "Test" mode): drive each rack motor ALONE to check direction / wiring
    // =====================================================================
    // 1. Robot on blocks, chains on. Driver Station -> Test -> Enable.
    // 2. Type a small number (1.0 to 1.5 V) in "Intake/Test/Leader 11 volts". Watch which way the intake goes.
    //    Set it back to 0.
    // 3. Same with "Intake/Test/Follower 14 volts".
    // 4. Expected: +volts on 11 = OUT, and -volts on 14 = OUT (14 is mounted facing the other way). Also check
    //    "Intake/Leader current (A)" and "Intake/Follower current (A)" go up when that motor is pushing.
    // Values are clamped to +/-3 V and drop to 0 when you leave Test mode.
    private static final double kMotorTestMaxVolts = 3.0;

    public void startMotorTest() {
        m_motorTest = true;
        SmartDashboard.putNumber("Intake/Test/Leader 11 volts", 0);
        SmartDashboard.putNumber("Intake/Test/Follower 14 volts", 0);
        RobotLog.event("Intake motor test started");
    }

    public void runMotorTest() {
        double leader = clampVolts(SmartDashboard.getNumber("Intake/Test/Leader 11 volts", 0));
        double follower = clampVolts(SmartDashboard.getNumber("Intake/Test/Follower 14 volts", 0));
        // Each motor on its own (the follower is temporarily NOT following)
        intakeMotor.setControl(m_homingRequest.withOutput(leader));
        intakeMotorTwo.setControl(new VoltageOut(follower).withEnableFOC(false));
    }

    public void endMotorTest() {
        m_motorTest = false;
        intakeMotor.setControl(m_stopRequest);
        intakeMotorTwo.setControl(new Follower(m_intakeId, kFollowerAlignment));
        m_state = State.NOT_HOMED;   // positions may have moved: re-zero on the next enable
        RobotLog.event("Intake motor test ended");
    }

    private static double clampVolts(double v) {
        return Math.max(-kMotorTestMaxVolts, Math.min(kMotorTestMaxVolts, v));
    }

    // =====================================================================
    // PID / speed tuning from the dashboard
    // =====================================================================
    // HOW TO TUNE (robot on blocks, chains on, no fuel):
    //   1. Put the intake at a known position, set Intake/Tune/kP = 0, kV = 0.
    //   2. kS: raise until the rack JUST starts to creep when given a small target change, then back off a bit.
    //   3. kV: with Motion Magic, raise until "Intake/Velocity (in per s)" matches the cruise speed mid-move.
    //   4. kP: raise until it lands on target quickly; stop when it overshoots, then back off ~20%.
    //   5. kD: only if it still bounces at the end of a move; add small amounts (0.05, 0.1...).
    //   6. Speeds: raise cruise/accel until it's as fast as you want without slamming.
    // Press "Intake/Tune/Apply" after changing numbers. Use "Intake/Tune/Go to test target" to try moves.
    // When it's good, copy the numbers into the kDefault... constants above so they stick.

    private void publishTuningControls() {
        SmartDashboard.putNumber("Intake/Tune/kP", kDefaultP);
        SmartDashboard.putNumber("Intake/Tune/kI", kDefaultI);
        SmartDashboard.putNumber("Intake/Tune/kD", kDefaultD);
        SmartDashboard.putNumber("Intake/Tune/kS", kDefaultS);
        SmartDashboard.putNumber("Intake/Tune/kV", kDefaultV);
        SmartDashboard.putNumber("Intake/Tune/kA", kDefaultA);
        SmartDashboard.putNumber("Intake/Tune/Cruise (in per s)", kDefaultCruiseInPerSec);
        SmartDashboard.putNumber("Intake/Tune/Accel (in per s2)", kDefaultAccelInPerSec2);
        SmartDashboard.putNumber("Intake/Tune/Test target (in)", EvilIntakePosition.out.getInches());
        SmartDashboard.putData("Intake/Tune/Apply", new InstantCommand(this::applyTuning).ignoringDisable(true));
        SmartDashboard.putData("Intake/Tune/Go to test target",
            run(() -> setTargetInches(SmartDashboard.getNumber("Intake/Tune/Test target (in)", m_targetInches)))
                .withName("Intake test target"));
        SmartDashboard.putData("Intake/Re-home", requestHomeCommand());
    }

    private static Slot0Configs gainsFromDashboardDefaults() {
        return new Slot0Configs()
            .withKP(kDefaultP).withKI(kDefaultI).withKD(kDefaultD)
            .withKS(kDefaultS).withKV(kDefaultV).withKA(kDefaultA);
    }

    /** Reads the Intake/Tune numbers and applies them (gains to both motors, speeds immediately) */
    private void applyTuning() {
        Slot0Configs gains = new Slot0Configs()
            .withKP(SmartDashboard.getNumber("Intake/Tune/kP", kDefaultP))
            .withKI(SmartDashboard.getNumber("Intake/Tune/kI", kDefaultI))
            .withKD(SmartDashboard.getNumber("Intake/Tune/kD", kDefaultD))
            .withKS(SmartDashboard.getNumber("Intake/Tune/kS", kDefaultS))
            .withKV(SmartDashboard.getNumber("Intake/Tune/kV", kDefaultV))
            .withKA(SmartDashboard.getNumber("Intake/Tune/kA", kDefaultA))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        intakeMotor.getConfigurator().apply(gains, 0.05);
        intakeMotorTwo.getConfigurator().apply(gains, 0.05);
        MotionMagicConfigs speeds = motionMagicFor(
            SmartDashboard.getNumber("Intake/Tune/Cruise (in per s)", kDefaultCruiseInPerSec),
            SmartDashboard.getNumber("Intake/Tune/Accel (in per s2)", kDefaultAccelInPerSec2));
        intakeMotor.getConfigurator().apply(speeds, 0.05);
        RobotLog.event(String.format("Intake tuning applied: kP %.3f kI %.3f kD %.3f kS %.3f kV %.3f kA %.3f",
            gains.kP, gains.kI, gains.kD, gains.kS, gains.kV, gains.kA));
    }
}
