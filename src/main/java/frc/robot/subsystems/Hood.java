package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    TalonFX hood;

    private double setpoint = 0.0;

    // --- SOFT LIMITS (PLACEHOLDERS, in motor rotations) ---
    // TODO: replace with the real numbers. Move the hood by hand to each hard stop with the robot
    // disabled, read "Hood Angle" on the dashboard, and put those values here minus a little margin.
    // The hood encoder zeroes wherever the hood is at power-on, so always boot with it at rest.
    // The aiming code already clamps to -2.77 .. -0.13, so these sit just outside that.
    private static final double kHoodForwardSoftLimit = 0.10;   // rest end (hood fully down)
    private static final double kHoodReverseSoftLimit = -2.90;  // most extended

    // --- TRENCH PROTECTION ---
    // A raised hood breaks on the trench (22in clearance). The hood rests here whenever we aren't shooting,
    // and is forced here near a trench even while shooting. Same resting value the aiming code uses.
    private static final double kHoodStowed = -0.12890625;
    /** Robot center to its farthest corner (0.64m) plus a little margin, in meters. Kept tight so shooting
     *  right beside the trench mouth (Double Swipe walk, 0.73m away) still works; the lookahead adds margin when moving. */
    public static final double kTrenchReachMeters = 0.68;
    /** Start lowering this many seconds before the robot would reach a trench at its current speed */
    public static final double kTrenchLookaheadSeconds = 0.5;
    /** Hood counts as down once it's within this many rotations of stowed */
    private static final double kStowedTolerance = 0.25;

    private final BooleanSupplier m_nearTrench;
    private boolean m_forcedDown = false;
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true);

    /** @param nearTrench true when the robot is in or about to enter a trench (see FieldZones) */
    public Hood(CANBus canbus, BooleanSupplier nearTrench){
        m_nearTrench = nearTrench;
        hood = new TalonFX(20, canbus);

        // --- HOOD CONFIG ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kHoodForwardSoftLimit;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kHoodReverseSoftLimit;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hood.getConfigurator().apply(hoodConfig);

        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = 2;
        hood.getConfigurator().apply(hoodPID);
    }

    /** Never true while the hood is being held down for a trench, so nothing feeds until it's back up */
    public boolean atSetpoint(){
        return !m_forcedDown && Math.abs(getAngle() - setpoint) <= 0.5;
    }

    /** @return true when the hood is down low enough to fit under the trench */
    public boolean isStowed(){
        return Math.abs(getAngle() - kHoodStowed) <= kStowedTolerance;
    }

    /** @return hood position */
    public double getAngle(){
       return hood.getPosition().getValueAsDouble();
    }

    // hood go go!
    public void goTo(double position){
        setpoint = position;
        applySetpoint();
    }
    
    public void incrementPositionBy(double revolutions) {
        setpoint += revolutions;
        applySetpoint();
    }

    /** Sends the setpoint, unless we're near a trench, then it holds the hood down instead */
    private void applySetpoint(){
        hood.setControl(m_request.withPosition(m_forcedDown ? kHoodStowed : setpoint));
    }

    /** Lowers the hood to its resting position */
    public void stow(){
        setpoint = kHoodStowed;
        applySetpoint();
    }

    public void stop(){
        stow();
    }

    /** Default command: hood stays down whenever nothing is shooting */
    public Command stowCommand() {
        return run(this::stow);
    }

    public Command hoodgo(DoubleSupplier posH) {
        return this.runEnd(
            () -> goTo(posH.getAsDouble()), 
            () -> stop()
        );
    }

    @Override
    public void periodic(){
        // Runs every loop before commands, so even a hood left up gets pulled down before a trench
        m_forcedDown = m_nearTrench.getAsBoolean();
        if (m_forcedDown) {
            hood.setControl(m_request.withPosition(kHoodStowed));
        }
        SmartDashboard.putBoolean("Hood/Forced Down (trench)", m_forcedDown);
        SmartDashboard.putBoolean("Hood/Stowed", isStowed());
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", setpoint);
    }


    
}
