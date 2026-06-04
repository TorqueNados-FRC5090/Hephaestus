package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class RollerSystem extends SubsystemBase {
    TalonFX rollerFloor; 
    TalonFX lowerBelt; 
    TalonFX upperBelt;
    public RollerSystem(){
        rollerFloor = new TalonFX(13, "Upper");
        lowerBelt = new TalonFX(21, "Upper");
        upperBelt = new TalonFX(22, "Upper");

        
        Slot0Configs rollerConfig = new Slot0Configs();
        rollerConfig.kP = 2;
        rollerFloor.getConfigurator().apply(rollerConfig);
        lowerBelt.getConfigurator().apply(rollerConfig);
        upperBelt.getConfigurator().apply(rollerConfig);

        lowerBelt.setControl(new Follower(13, MotorAlignmentValue.Aligned));
        upperBelt.setControl(new Follower(13, MotorAlignmentValue.Aligned));

    } 

    // sets speed
    public void roll(double rollerSpeed){
        VelocityVoltage velocityRequest = new VelocityVoltage(rollerSpeed).withSlot(0);
        rollerFloor.setControl(velocityRequest);
    }

    // sets speed to 0
    public void rollerStop(){
        VelocityVoltage stopRequest = new VelocityVoltage(0).withSlot(0);
        rollerFloor.setControl(stopRequest);
    } 

    public void unjam(){
        VelocityVoltage unjamRequest = new VelocityVoltage(-1).withSlot(0);
        rollerFloor.setControl(unjamRequest);
    }

    public Command otherUnjam(){
        return runEnd(
            () -> roll(-1), 
            () -> rollerStop()
        );
    }

     @Override
    public void periodic() {
        // SmartDashboard.putNumber("accellator", accelterator.getPosition().getValueAsDouble());
    }
}
