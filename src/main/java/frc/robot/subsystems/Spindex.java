package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Spindex extends SubsystemBase {
    TalonFX spindex; 
    TalonFX accelterator; 
    public Spindex(){
        spindex = new TalonFX(13, "Upper");
        accelterator = new TalonFX(14, "Upper");
        
        Slot0Configs spinConfig = new Slot0Configs();
        spinConfig.kP = .1;
        accelterator.getConfigurator().apply(spinConfig);
        spindex.getConfigurator().apply(spinConfig);
        
        
        accelterator.setControl(new Follower(13, MotorAlignmentValue.Aligned));
    }
//Set Speed and Acceleration
    public void spin(double spinspeed){
        spindex.set(spinspeed);

    }
//Sets Speed and Acceleration to
    public void spindexStop(){
        spindex.set(0);
    } 
     @Override
    public void periodic() {
        SmartDashboard.putNumber("accellator", accelterator.getPosition().getValueAsDouble());
    }
}