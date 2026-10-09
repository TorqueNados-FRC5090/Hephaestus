package frc.robot.subsystems;

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

    public Hood(CANBus canbus){
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

    public boolean atSetpoint(){
        return Math.abs(getAngle() - setpoint) <= 0.5;
    }

    /** @return hood position */
    public double getAngle(){
       return hood.getPosition().getValueAsDouble();
    }

    // hood go go!
    public void goTo(double position){
        setpoint = position;
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0).withEnableFOC(true);
        hood.setControl(hoodRequest);
    }
    
    public void incrementPositionBy(double revolutions) {
        setpoint += revolutions;
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0).withEnableFOC(true);
        hood.setControl(hoodRequest);
    }

    public void stop(){
        hood.set(0);
    }

    public Command hoodgo(DoubleSupplier posH) {
        return this.runEnd(
            () -> goTo(posH.getAsDouble()), 
            () -> stop()
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", setpoint);
    }


    
}
