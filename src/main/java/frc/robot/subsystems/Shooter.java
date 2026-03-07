package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.ShooterPosition;

public class Shooter extends SubsystemBase {
  TalonFX hood;
  TalonFX leadShoot;
  TalonFX followShoot;
  ShooterPosition pos = ShooterPosition.zero;
    public Shooter(){
        hood = new TalonFX(20, "Upper");
        leadShoot = new TalonFX(27, "Upper");
        followShoot = new TalonFX(15,"Upper");
    

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        
        hood.getConfigurator().apply(hoodConfig);
        

        Slot0Configs PIDconfig = new Slot0Configs();
        PIDconfig.kP = .6;
        hood.getConfigurator().apply(PIDconfig);
        leadShoot.getConfigurator().apply(PIDconfig);
        followShoot.getConfigurator().apply(PIDconfig);

        
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));
        


    }

    public boolean atSetpoint(){
        return Math.abs(getAngle() - pos.getAngle()) <= .5;
    }

    //gets hood position

    public double getAngle(){
       return hood.getPosition().getValueAsDouble();
    }

    //hood go go!
    public void setTarget(ShooterPosition target){
        pos = target;
        PositionVoltage hoodRequest = new PositionVoltage(target.getAngle()).withSlot(0);
        hood.setControl(hoodRequest);
    }
    //shoots
    public void goShoot(){
        VelocityVoltage velocityRequest = new VelocityVoltage(100).withSlot(0);
        leadShoot.setControl(velocityRequest);
    }
    //gets angle of target (NOT HOOD ANGEL)
    public double getTarget(ShooterPosition target){
        return target.getAngle();
    }
    //move hood to any position
    public void goHood(){
        hood.set(.1);
        
    }
      
    public void plusAngle(double angle){
        PositionVoltage hoodBump = new PositionVoltage(angle);
        hood.setControl(hoodBump);
        
    }
    public void plusVelocity(){
        PositionVoltage velocityBump = new PositionVoltage(leadShoot.getVelocity().getValueAsDouble() + 100);
        leadShoot.setControl(velocityBump);
    }
//
    public void stop(){
        hood.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", pos.getAngle());
        SmartDashboard.putNumber("Velocity", leadShoot.getVelocity().getValueAsDouble());
    }


    
}
