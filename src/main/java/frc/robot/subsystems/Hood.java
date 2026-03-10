package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    TalonFX hood;

    private double setpoint = 0.0;

    public Hood(){
        hood = new TalonFX(20, "Upper");

        // --- HOOD CONFIG ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hood.getConfigurator().apply(hoodConfig);

        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = 0.5;
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
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0);
        hood.setControl(hoodRequest);
    }
    
    public void incrementPositionBy(double revolutions) {
        setpoint += revolutions;
        PositionVoltage hoodRequest = new PositionVoltage(setpoint).withSlot(0);
        hood.setControl(hoodRequest);
    }

    public void stop(){
        hood.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", setpoint);
    }


    
}
