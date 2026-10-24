package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
 * plus two Kraken X60 intake wheels (Talon FX) facing each other, so they spin toward each other to grip
 * fuel between them. The second wheel (10/23) is a real CTRE Follower (Opposed) of the first -- unlike the
 * rack motors, the wheels just need to spin together, not hold independent positions, so one command
 * (spinRequest, see below) drives both with no extra code.
 *
 * <p>How it works:
 * <ul>
 *   <li>Motors 11 and 14 each run their OWN position control (Motion Magic) to the same target, each on its own
 *       encoder. A plain follower only copies power, so one side lagged ~1" when retracting, the intake cocked and
 *       bound 3" short of in (10/21 log). Positive = OUT on both: 14 is inverted in its config because it faces 11.</li>
 *   <li>Every move uses Motion Magic, so speed and acceleration are always limited (unlimited, this gearing
 *       would slam the rack at ~200 in/s). Standard Motion Magic on purpose: it works without a Phoenix Pro
 *       license (Pro-only controls make an unlicensed Talon refuse to move).</li>
 *   <li>Zero: at power-on the intake must be fully in (0"). It NEVER homes by itself (the automatic homing on
 *       enable caused trouble on 10/21). Homing only runs from D-pad up / dashboard "Intake/Re-home": both sides
 *       push IN against the retract stop and each side is zeroed at 0" directly when it stops there (squares
 *       the intake). Pushing in rather than out on purpose (10/22): zero is defined right where it's measured
 *       instead of backed out from the far end, and the retract side is just the robot's own frame, not
 *       whatever's in front of the intake. If homing fails it falls back to the power-on zero and keeps working.</li>
 *   <li>Sync: each side gets a small extra push/hold-back based on how far it is ahead of the other side, so the
 *       side with less friction can't run ahead (10/21 log: one side got ~3" ahead, the intake twisted).</li>
 *   <li>A low stator current limit makes the rack act like a force-capped spring: a hit pushes it in instead of
 *       skipping the chain, then Motion Magic brings it back.</li>
 *   <li>Sync guard: if the two sides drift more than 1" apart (one side blocked), both stop until re-homed.</li>
 * </ul>
 * Commands only set a target; periodic() is the one place that talks to the rack motors.
 */
public class EvilIntake extends SubsystemBase {
    // ================= GEARING / TRAVEL =================
    /** Vortex 14T -> 22T chain (1.57:1) */
    private static final double kChainRatio = 22.0 / 14.0;
    /** Planetary gearbox between each Vortex and its 14T sprocket (5:1 added 10/21). 1.0 = none. */
    private static final double kPlanetaryRatio = 5.0;
    /** 10T 10DP pinion on a 10DP rack: pitch diameter = 10 / 10 = 1.0" */
    private static final double kPinionPitchDiameterInches = 1.0;
    /** Motor rotations per inch of rack travel: 1.57 / 3.14 = 0.500 (2.5 with a 5:1 planetary) */
    public static final double kMotorRotationsPerInch =
        kPlanetaryRatio * kChainRatio / (Math.PI * kPinionPitchDiameterInches);
    /** Extension hard stop (inches of rack travel from fully in). MEASURED 10/21: both sides stalled solid at
     *  6.06 motor rotations = 12.1" (the rack is 17" long, but the travel is shorter). Re-measure with a tape and
     *  fix this if it's different. Homing pushes against this and sets it as the reference. */
    private static final double kExtendStopInches = 12.1;
    /** Never command closer than this to the extension stop (Motion Magic would push into it) */
    private static final double kMaxCommandInches = 11.75;
    /** Never command into the retract stop (fuel gets squeezed / jams) */
    private static final double kMinCommandInches = 0.2;

    // ================= SPEED (Dynamic Motion Magic: separate profile for deploy vs retract) =================
    // Change these live from the dashboard (Intake/Tune) and copy the good values back here.
    // Deploying is gentler on purpose: going out pins both motors near the current limit for real mechanical
    // resistance, so asking for less acceleration there means less torque demanded and it doesn't stutter.
    // Retracting doesn't fight that resistance, so it keeps the snappier profile.
    private static final double kRetractCruiseInPerSec = 25.0;   // full 11" stroke in ~0.6 s
    private static final double kRetractAccelInPerSec2 = 100.0;
    private static final double kDeployCruiseInPerSec = 14.0;    // full 11" stroke in ~1.1 s
    private static final double kDeployAccelInPerSec2 = 35.0;
    // Agitate follows a slow back-and-forth (kAgitatePeriodSeconds), always slower than the cruise speed.

    // ================= AUTO ZERO (hard-stop homing) =================
    /** Volts pushing OUT while homing. Starts at 2 V (~7 in/s with the 5:1) and steps up if the rack doesn't move. */
    private static final double kHomingStartVolts = 2.0;
    private static final double kHomingMaxVolts = 4.0;
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
    private static final double kHomingTimeoutSeconds = 5.0;

    // ================= KEEPING THE SIDES TOGETHER =================
    /** Extra volts per inch one side is ahead of the average: the side ahead is held back, the side behind is
     *  pushed harder. Bigger = tighter together (too big = buzzing/shaking). */
    private static final double kSyncVoltsPerInch = 2.0;   // lowered for the 5:1 (each volt pushes 5x harder)
    /** Cap on that extra push, in volts */
    private static final double kSyncMaxVolts = 3.0;

    // ================= SYNC GUARD =================
    /** If the two sides disagree by more than this (inches) for kSyncFaultSeconds, both stop until re-homed */
    private static final double kSyncFaultInches = 1.0;
    private static final double kSyncFaultSeconds = 0.25;
    /** false = only warn (dashboard + log). Set true once "Intake/Sides differ (in)" has been seen to stay near
     *  0 during normal moves (proves the follower reading is the right way round). */
    private static final boolean kSyncGuardStopsIntake = false;

    // ================= CURRENT LIMITS (why) =================
    /** Rack motors, stator: the actual push force. 24 A with the 5:1 planetary is roughly 47 lb/side (94 lb
     *  both) -- enough to move the rack without the chain/pinion feeling excessive force. Lower = more give
     *  when the intake gets hit; don't go much above 25 A or the chain takes all of it. */
    private static final double kRackStatorLimitAmps = 24.0;
    /** Rack motors, supply: caps battery draw per side (separate from stator -- this is what trims brownouts,
     *  not push force). Trimmed down over several brownout-chasing sessions; raise it if the rack ever feels
     *  starved once the battery itself checks out healthy. */
    private static final double kRackSupplyLimitAmps = 15.0;
    /** Intake wheels, stator: enough grip to pull fuel in; limits heat if the wheels stall on a jam */
    private static final double kWheelStatorLimitAmps = 40.0;
    /** Intake wheels, supply: battery protection, same reasoning as the rack's supply limit above. */
    private static final double kWheelSupplyLimitAmps = 15.0;

    // ================= GAINS (starting values, tune on the dashboard) =================
    // Units: volts per motor rotation of error (kP), volts per rot/s (kV), volts (kS).
    // kI and kA aren't exposed for tuning: this mechanism has never needed either (Motion Magic handles the
    // acceleration feedforward on its own, and there's been no steady-state error worth an integral term).
    private static final double kDefaultP = 1.5;   // pushes harder the further it is from target (1" off = 2.5 rot = ~3.8 V)
    private static final double kDefaultD = 0.0;   // add a little if it overshoots or bounces
    private static final double kDefaultS = 0.25;  // volts to get past friction
    private static final double kDefaultV = 0.11;  // ~12 V / Vortex free speed (113 rot/s)

    // ================= WHEELS / AGITATE =================
    /** Roller power while intaking. Negative = pulls fuel in (same as EvilIntakePiece). */
    private static final double kRollerIntakeSpeed = -1.0;
    /** Roller power while ejecting (spits fuel back out the intake) */
    private static final double kRollerEjectSpeed = 1.0;
    /** How far in the rack slides during agitate (inches). Only part way: all the way in with fuel jams it. */
    private static final double kAgitateInInches = 5.7;
    /** Seconds for one full out -> in -> out cycle. Bigger = slower */
    private static final double kAgitatePeriodSeconds = 2.0;

    // ================= DIRECTIONS (both: positive = intake OUT) =================
    /** Motor 11: +volts moved the intake out on the robot (10/21) */
    private static final InvertedValue kMotor11Inverted = InvertedValue.CounterClockwise_Positive;
    /** Motor 14 faces motor 11, so it's inverted the other way. With Aligned-follower it drove its side IN. */
    private static final InvertedValue kMotor14Inverted = InvertedValue.Clockwise_Positive;

    // ================= HARDWARE =================
    final TalonFXS intakeMotor;      // leader (CAN 11)
    final TalonFXS intakeMotorTwo;   // follower (CAN 14), opposed
    final TalonFX spinMotor;         // intake wheels, leader (CAN 12)
    final TalonFX spinMotorTwo;      // intake wheels #2, follower (10/23: second Kraken X60, spins opposed)

    // FOC ON: 11 and 14 are Phoenix Pro licensed (confirmed in the 10/21 19:49 log). ~15% more torque.
    // If the licenses ever lapse, set these to false or the Talons stop driving (unlicensed fault).
    private static final boolean kUseFOC = true;
    // Dynamic Motion Magic: lets us pick cruise/accel per move (deploy vs retract) without re-flashing configs
    private final DynamicMotionMagicVoltage m_moveRequest = new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(kUseFOC);
    private final DynamicMotionMagicVoltage m_moveRequest14 = new DynamicMotionMagicVoltage(0, 0, 0).withEnableFOC(kUseFOC);
    private final VoltageOut m_homingRequest = new VoltageOut(0).withEnableFOC(kUseFOC);
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
    private final Timer[] m_stallTimer = { new Timer(), new Timer() };   // [0] = motor 11, [1] = motor 14
    private final Timer m_syncTimer = new Timer();
    private double m_targetInches = EvilIntakePosition.in.getInches();
    private final double[] m_homingStartInches = new double[2];
    private final boolean[] m_homingMoved = new boolean[2];
    private final boolean[] m_sideHomed = new boolean[2];
    private double m_homingVolts = kHomingStartVolts;
    private boolean m_motorTest = false;
    /** True while either rack motor is pinned at its stator current limit (can't keep up with the profile) */
    private boolean m_atCurrentLimit = false;
    // Live cruise/accel, tunable separately per direction from Intake/Tune (see driveToTarget)
    private double m_deployCruiseInPerSec = kDeployCruiseInPerSec;
    private double m_deployAccelInPerSec2 = kDeployAccelInPerSec2;
    private double m_retractCruiseInPerSec = kRetractCruiseInPerSec;
    private double m_retractAccelInPerSec2 = kRetractAccelInPerSec2;

    /** Constructs the intake
     * @param intakeID CAN ID of the leader rack motor (Talon FXS + Vortex), positive = out
     * @param intakeTwoID CAN ID of the follower rack motor (Talon FXS + Vortex), mounted opposite
     * @param spinID CAN ID of the leader intake wheel motor (Talon FX, Kraken X60)
     * @param spinTwoID CAN ID of the second intake wheel motor (Talon FX, Kraken X60), mounted facing the
     *                  other way so the two wheels spin toward each other to grip fuel between them
     */
    public EvilIntake(int intakeID, int intakeTwoID, int spinID, int spinTwoID, CANBus canbus){
        intakeMotor = new TalonFXS(intakeID, canbus);
        intakeMotorTwo = new TalonFXS(intakeTwoID, canbus);
        spinMotor = new TalonFX(spinID, canbus);
        spinMotorTwo = new TalonFX(spinTwoID, canbus);

        // --- RACK MOTORS (Talon FXS + NEO Vortex) ---
        TalonFXSConfiguration rackConfig = new TalonFXSConfiguration();
        // Tells the Talon FXS a Vortex is attached (through the REV Vortex Solo Adapter). Nothing runs without it.
        rackConfig.Commutation.MotorArrangement = MotorArrangementValue.VORTEX_JST;
        // Better commutation + speed measurement for hall-sensor motors like the Vortex (Phoenix Pro: licensed now)
        rackConfig.Commutation.AdvancedHallSupport = kUseFOC ? AdvancedHallSupportValue.Enabled : AdvancedHallSupportValue.Disabled;
        rackConfig.MotorOutput.Inverted = kMotor11Inverted;
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
        // No MotionMagicConfigs here on purpose: Dynamic Motion Magic (below) sets cruise/accel/jerk on every
        // control call instead, so it can use a different profile for deploying vs retracting.
        // Push toward the target to get past friction (kS sign follows the error)
        rackConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        intakeMotor.getConfigurator().apply(rackConfig);
        rackConfig.MotorOutput.Inverted = kMotor14Inverted;   // same settings, mirrored direction
        intakeMotorTwo.getConfigurator().apply(rackConfig);
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
        // Second wheel motor (10/23): same limits, just follows motor 12 opposed so one command (spinRequest
        // below) drives both. Unlike the rack motors, the wheels don't need independent position control or a
        // sync correction -- it's a plain velocity spinner, so a real Follower is the simple, correct choice here.
        spinMotorTwo.getConfigurator().apply(spinConfig);
        spinMotorTwo.setControl(new Follower(spinID, MotorAlignmentValue.Opposed));

        publishTuningControls();
    }

    // =====================================================================
    // Commands / API used by the rest of the code (names kept the same)
    // =====================================================================

    /** Move the rack to a named position (normal speed) */
    public void evilyummy(EvilIntakePosition pos){
        setTargetInches(pos.getInches());
    }

    /** Intake wheel power, -1..1 (negative pulls fuel in) */
    public void evileryummy(double speed){
        spinMotor.setControl(spinRequest.withOutput(speed));
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

    /** @return motor 14's side, in inches of rack travel */
    private double getInches14(){
        return intakeMotorTwo.getPosition().getValueAsDouble() / kMotorRotationsPerInch;
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
            publishTelemetry(getInches(), getInches14(),
                intakeMotor.getVelocity().getValueAsDouble());
            return;   // Test mode: runMotorTest() is driving the motors, don't touch them here
        }
        double leaderInches = getInches();
        double followerInches = getInches14();   // both read "inches out" (14 is inverted in its config)
        double velocityRps = intakeMotor.getVelocity().getValueAsDouble();

        if (DriverStation.isDisabled() && m_state == State.HOMING) {
            m_state = State.NOT_HOMED;   // disabled mid-homing: start over on the next enable
        }

        // 10/21: a chain came off one side and the zero silently drifted (position read -2.9" at one point,
        // which is physically impossible if 0 = fully in). Nothing caught that live -- it only showed up when
        // the logs were scanned by hand afterward. Flag it the moment it happens instead.
        double outOfRangeInches = 1.0;   // some slop for normal overshoot/backlash, not a hair-trigger
        boolean outOfRange = leaderInches < -outOfRangeInches || followerInches < -outOfRangeInches
            || leaderInches > kExtendStopInches + outOfRangeInches
            || followerInches > kExtendStopInches + outOfRangeInches;
        // Lazy message (10/23): String.format was running every 20 ms loop even when nothing was wrong --
        // this is one of the fixes for that day's loop overruns. Only actually builds the string on the edge.
        RobotLog.eventOnRise("intakeOutOfRange", outOfRange, () -> String.format(
            "Intake position OUT OF RANGE (%.2f in / %.2f in, expected 0-%.1f) -- chain skipped or lost its zero, re-home",
            leaderInches, followerInches, kExtendStopInches));

        if (DriverStation.isEnabled()) {
            switch (m_state) {
                case NOT_HOMED -> {
                    if (m_homeRequested) {
                        startHoming();   // only when asked: D-pad up / dashboard Re-home
                    } else {
                        // Use the power-on zero (intake must be fully in when the robot is turned on)
                        m_state = State.READY;
                        m_syncTimer.restart();
                    }
                }
                case HOMING -> runHoming(velocityRps);
                case READY -> {
                    checkSync(leaderInches, followerInches);
                    if (m_state == State.READY) {
                        driveToTarget(leaderInches, followerInches);
                    }
                }
                // Out of sync: hold still until someone re-homes (D-pad up or dashboard)
                case SIDES_OUT_OF_SYNC -> {
                    intakeMotor.setControl(m_stopRequest);
                    intakeMotorTwo.setControl(m_stopRequest);
                }
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
        // 10/24: Michel noticed one rack controller running hot. This wasn't tracked anywhere before --
        // watch these against the DeviceTemp fault above (CTRE's Talon FXS trips that one in the 80s C).
        SmartDashboard.putNumber("Intake/Leader temp (C)", intakeMotor.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Follower temp (C)", intakeMotorTwo.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Leader volts", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Roller voltage", spinMotor.getMotorVoltage().getValueAsDouble());
        // Wheel current + velocity (10/23): voltage alone can't tell "spinning freely" apart from "stalled" --
        // a stalled Kraken/Falcon can show the same low volts a lightly-loaded free-spinning one does. Current
        // pegged at/near the limit while velocity sits near 0 is the stall signature (same as the floor roller).
        SmartDashboard.putNumber("Intake/Wheel current (A)", spinMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Wheel velocity (rps)", spinMotor.getVelocity().getValueAsDouble());
        // Second wheel motor (10/23): it's a Follower electrically, but it's still a separate physical motor
        // that can jam/fault on its own while the leader spins fine -- worth watching independently.
        SmartDashboard.putNumber("Intake/Wheel 2 current (A)", spinMotorTwo.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Wheel 2 velocity (rps)", spinMotorTwo.getVelocity().getValueAsDouble());
        // Anything stopping a rack Talon FXS from driving, right now (not sticky)
        String faults = activeFaults("leader", intakeMotor) + activeFaults("follower", intakeMotorTwo)
            + wheelFaults("wheels", spinMotor) + wheelFaults("wheels 2", spinMotorTwo);
        SmartDashboard.putString("Intake/Faults", faults.isEmpty() ? "none" : faults);
        // True = the motors can't keep up with the commanded move without hitting their current limit (expect
        // this briefly on hard starts; if it's on for a long stretch mid-move, that's real resistance to go look at)
        SmartDashboard.putBoolean("Intake/At current limit", m_atCurrentLimit);
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
        // 10/24: Michel noticed one rack controller running hot. Neither of these was being checked before --
        // DeviceTemp is the Talon FXS board itself overheating (likely, given how often these sit pinned near
        // their current limit); ProcTemp is its internal CPU. Both cut the Talon's output when tripped.
        if (talon.getFault_DeviceTemp().getValue()) f.append(name).append(": CONTROLLER OVERHEATED (DeviceTemp). ");
        if (talon.getFault_ProcTemp().getValue()) f.append(name).append(": controller CPU overheated (ProcTemp). ");
        return f.toString();
    }

    /** Same idea as activeFaults() above, but for the wheel motor (a plain TalonFX, no hall-sensor port) */
    private static String wheelFaults(String name, TalonFX talon) {
        StringBuilder f = new StringBuilder();
        if (talon.getFault_Hardware().getValue()) f.append(name).append(": HARDWARE fault. ");
        if (talon.getFault_BridgeBrownout().getValue()) f.append(name).append(": bridge brownout. ");
        if (talon.getFault_Undervoltage().getValue()) f.append(name).append(": low voltage. ");
        return f.toString();
    }

    private void driveToTarget(double inches11, double inches14) {
        // Same target to both sides, each closing the loop on its own encoder, plus a correction that holds back
        // the side that's ahead and pushes the side that's behind. Keeps the intake square.
        double ahead11 = (inches11 - inches14) / 2.0;   // how far 11 is ahead of the middle (14 is the opposite)
        double sync = Math.max(-kSyncMaxVolts, Math.min(kSyncMaxVolts, kSyncVoltsPerInch * ahead11));
        double targetRotations = m_targetInches * kMotorRotationsPerInch;

        // Gentler profile going out (see kDeployCruiseInPerSec above): asking for less acceleration means less
        // torque demanded, so the motors don't have to hit the current limit just to keep up with the profile.
        boolean deploying = m_targetInches > (inches11 + inches14) / 2.0;
        double cruiseInPerSec = deploying ? m_deployCruiseInPerSec : m_retractCruiseInPerSec;
        double accelInPerSec2 = deploying ? m_deployAccelInPerSec2 : m_retractAccelInPerSec2;
        // Telemetry only -- dashboard-informational, doesn't affect control -- so only read it on the 10 Hz
        // tick instead of every 20 ms loop (10/23 loop-overrun pass: 2 fewer fault reads every loop).
        if (RobotLog.telemetryTick()) {
            m_atCurrentLimit = intakeMotor.getFault_StatorCurrLimit().getValue()
                || intakeMotorTwo.getFault_StatorCurrLimit().getValue();
        }

        intakeMotor.setControl(m_moveRequest.withPosition(targetRotations)
            .withVelocity(cruiseInPerSec * kMotorRotationsPerInch)
            .withAcceleration(accelInPerSec2 * kMotorRotationsPerInch)
            .withJerk(accelInPerSec2 * kMotorRotationsPerInch * 10)
            .withFeedForward(-sync));
        intakeMotorTwo.setControl(m_moveRequest14.withPosition(targetRotations)
            .withVelocity(cruiseInPerSec * kMotorRotationsPerInch)
            .withAcceleration(accelInPerSec2 * kMotorRotationsPerInch)
            .withJerk(accelInPerSec2 * kMotorRotationsPerInch * 10)
            .withFeedForward(sync));
    }

    private void startHoming() {
        m_state = State.HOMING;
        m_homingTimer.restart();
        m_homingStartInches[0] = getInches();
        m_homingStartInches[1] = getInches14();
        for (int side = 0; side < 2; side++) {
            m_stallTimer[side].restart();
            m_homingMoved[side] = false;
            m_sideHomed[side] = false;
        }
        m_homingVolts = kHomingStartVolts;
        // 10/22: homes against the RETRACT stop now (0"), not the extension stop -- zero is defined directly
        // where it's measured instead of being backed out from the far end, and pushing in (against the frame)
        // is also the safer direction to be shoving into blind (the extension side could have fuel/field stuff
        // in the way; the retract side is just the robot's own frame).
        RobotLog.event("Intake homing: pushing both sides in to the retract stop (zero)");
    }

    /** Both sides push IN gently; each side is zeroed at 0" on its own when IT stops at the retract stop. If
     *  one side gets there first, the other keeps pushing until it arrives too, which squares the intake. */
    private void runHoming(double velocityRps) {
        TalonFXS[] motors = { intakeMotor, intakeMotorTwo };
        double[] inches = { getInches(), getInches14() };
        double[] speeds = { velocityRps, intakeMotorTwo.getVelocity().getValueAsDouble() };
        double[] currents = { Math.abs(intakeMotor.getStatorCurrent().getValueAsDouble()),
                               Math.abs(intakeMotorTwo.getStatorCurrent().getValueAsDouble()) };
        boolean anyMoved = m_homingMoved[0] || m_homingMoved[1];
        if (!anyMoved && m_homingTimer.get() > kHomingIgnoreSeconds) {
            // Not moving yet (friction / chain): push a little harder, up to kHomingMaxVolts
            m_homingVolts = Math.min(kHomingMaxVolts, m_homingVolts + kHomingRampVoltsPerSec * 0.02);
        }
        for (int side = 0; side < 2; side++) {
            // Moving IN: position counts down from wherever it started, toward 0
            if (m_homingStartInches[side] - inches[side] >= kHomingMinTravelInches) {
                m_homingMoved[side] = true;
            }
            boolean moving = Math.abs(speeds[side]) > kHomingStallRps;
            // Pushing hard with no movement counts as reaching the stop too, not just "moved far, then
            // stopped" -- otherwise starting homing already near zero (barely any travel left) can never
            // satisfy kHomingMinTravelInches even though it's genuinely jammed against the stop.
            boolean pushingHard = currents[side] > kRackStatorLimitAmps * 0.6;
            boolean reachedSomething = m_homingMoved[side] || pushingHard;
            if (moving || !reachedSomething || m_homingTimer.get() < kHomingIgnoreSeconds) {
                m_stallTimer[side].restart();
            }
            if (!m_sideHomed[side] && m_stallTimer[side].get() >= kHomingStallSeconds) {
                motors[side].setPosition(0);   // this side is at the retract stop: that's zero, directly
                m_sideHomed[side] = true;
            }
            // Keep pushing gently inward (also once homed) so the other side can catch up without twisting
            motors[side].setControl(m_homingRequest.withOutput(-m_homingVolts));
        }
        SmartDashboard.putNumber("Intake/Homing volts", m_homingVolts);

        if (m_sideHomed[0] && m_sideHomed[1]) {
            m_syncTimer.restart();
            m_state = State.READY;
            m_homeRequested = false;
            m_zeroSource = "hard stop (homed, both sides)";
            RobotLog.event(String.format("Intake homed in %.2f s (both sides)", m_homingTimer.get()));
        } else if (m_homingTimer.get() > kHomingTimeoutSeconds) {
            // Don't lock the intake up for the rest of the match: keep what we have and carry on
            m_syncTimer.restart();
            m_state = State.READY;
            m_homeRequested = false;
            String why;
            if (m_sideHomed[0] || m_sideHomed[1]) {
                why = "only motor " + (m_sideHomed[0] ? "11" : "14") + "'s side reached the stop (other side binding?)";
            } else if (anyMoved) {
                why = "moved but never stopped at the retract stop (chain skipping? stop missing?)";
            } else {
                why = "never moved even at " + kHomingMaxVolts + " V (direction? motor/adapter? license? chain off?)";
            }
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
                intakeMotorTwo.setControl(m_stopRequest);
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
    // 4. Expected: +volts = OUT on BOTH (14's mirrored mounting is handled by its inverted config). Also check
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
        // Each motor on its own
        intakeMotor.setControl(new VoltageOut(leader).withEnableFOC(kUseFOC));
        intakeMotorTwo.setControl(new VoltageOut(follower).withEnableFOC(kUseFOC));
    }

    public void endMotorTest() {
        m_motorTest = false;
        intakeMotor.setControl(m_stopRequest);
        intakeMotorTwo.setControl(m_stopRequest);
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
        SmartDashboard.putNumber("Intake/Tune/kD", kDefaultD);
        SmartDashboard.putNumber("Intake/Tune/kS", kDefaultS);
        SmartDashboard.putNumber("Intake/Tune/kV", kDefaultV);
        SmartDashboard.putNumber("Intake/Tune/Deploy cruise (in per s)", kDeployCruiseInPerSec);
        SmartDashboard.putNumber("Intake/Tune/Deploy accel (in per s2)", kDeployAccelInPerSec2);
        SmartDashboard.putNumber("Intake/Tune/Retract cruise (in per s)", kRetractCruiseInPerSec);
        SmartDashboard.putNumber("Intake/Tune/Retract accel (in per s2)", kRetractAccelInPerSec2);
        SmartDashboard.putNumber("Intake/Tune/Test target (in)", EvilIntakePosition.out.getInches());
        SmartDashboard.putData("Intake/Tune/Apply", new InstantCommand(this::applyTuning).ignoringDisable(true));
        SmartDashboard.putData("Intake/Tune/Go to test target",
            run(() -> setTargetInches(SmartDashboard.getNumber("Intake/Tune/Test target (in)", m_targetInches)))
                .withName("Intake test target"));
        SmartDashboard.putData("Intake/Re-home", requestHomeCommand());
    }

    private static Slot0Configs gainsFromDashboardDefaults() {
        return new Slot0Configs()
            .withKP(kDefaultP).withKD(kDefaultD).withKS(kDefaultS).withKV(kDefaultV);
    }

    /** Reads the Intake/Tune numbers and applies them (gains to both motors, speeds immediately) */
    private void applyTuning() {
        Slot0Configs gains = new Slot0Configs()
            .withKP(SmartDashboard.getNumber("Intake/Tune/kP", kDefaultP))
            .withKD(SmartDashboard.getNumber("Intake/Tune/kD", kDefaultD))
            .withKS(SmartDashboard.getNumber("Intake/Tune/kS", kDefaultS))
            .withKV(SmartDashboard.getNumber("Intake/Tune/kV", kDefaultV))
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        intakeMotor.getConfigurator().apply(gains, 0.05);
        intakeMotorTwo.getConfigurator().apply(gains, 0.05);
        m_deployCruiseInPerSec = SmartDashboard.getNumber("Intake/Tune/Deploy cruise (in per s)", kDeployCruiseInPerSec);
        m_deployAccelInPerSec2 = SmartDashboard.getNumber("Intake/Tune/Deploy accel (in per s2)", kDeployAccelInPerSec2);
        m_retractCruiseInPerSec = SmartDashboard.getNumber("Intake/Tune/Retract cruise (in per s)", kRetractCruiseInPerSec);
        m_retractAccelInPerSec2 = SmartDashboard.getNumber("Intake/Tune/Retract accel (in per s2)", kRetractAccelInPerSec2);
        RobotLog.event(String.format("Intake tuning applied: kP %.3f kD %.3f kS %.3f kV %.3f",
            gains.kP, gains.kD, gains.kS, gains.kV));
    }
}
