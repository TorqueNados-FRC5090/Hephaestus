package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    }
//Set Speed and Acceleration
    public void spin(double spinspeed, double accelspeed){
        spindex.set(spinspeed);
        accelterator.set(accelspeed);
    }
//Sets Speed and Acceleration to
    public void spindexStop(){
        spindex.set(0);
        accelterator.set(0);
    } 

    public Command unjam() {
        return runEnd(
            () -> spin(0, .5), 
            () -> spindexStop()
        );
    }

     @Override
    public void periodic() {
        SmartDashboard.putNumber("accellator", accelterator.getPosition().getValueAsDouble());
    }
}
