package frc.robot.subsystems;

import java.util.function.ToDoubleFunction;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.ShooterPosition;

public class Shooter extends SubsystemBase {
    TalonFX hood;
    TalonFX leadShoot;
    TalonFX followShoot;
    ShooterPosition pos = ShooterPosition.zero;
    
    // We store the target RPS here so the plus/minus buttons work correctly
    private double targetShooterRPS = 0.0; 

    public Shooter(){
        hood = new TalonFX(20, "Upper");
        leadShoot = new TalonFX(27, "Upper");
        followShoot = new TalonFX(15,"Upper");
    
        // --- HOOD CONFIG (UNTOUCHED) ---
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hood.getConfigurator().apply(hoodConfig);

        Slot0Configs hoodPID = new Slot0Configs();
        hoodPID.kP = 0.5;
        hood.getConfigurator().apply(hoodPID);
        // -------------------------------

        // --- SHOOTER CONFIG ---
        Slot0Configs shooterPID = new Slot0Configs();
        shooterPID.kP = 2.0; // Changed to 2.0 for smoother recovery
        shooterPID.kI = 0.0;
        shooterPID.kD = 0.0;
        shooterPID.kV = 0.12;
        shooterPID.kS = 0.0;
        shooterPID.kA = 1.0;
        
        leadShoot.getConfigurator().apply(shooterPID);
        followShoot.getConfigurator().apply(shooterPID);
        
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));
    }

    public boolean atSetpoint(){
        return Math.abs(getAngle() - pos.getAngle()) <= .5;
    }

    // Checks if the shooter is up to speed and ready to fire
    public boolean isFlywheelReady(){
        // If we aren't trying to shoot, the flywheel isn't "ready"
        if (targetShooterRPS == 0.0) {
            return false;
        }

        // Get the current speed
        double currentRPS = leadShoot.getVelocity().getValueAsDouble();

        // Check if the current speed is within 2 RPS of our target
        return Math.abs(currentRPS - targetShooterRPS) <= 2.0;
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
    
    //shoots using the stored targetRPS variable
    public void goShoot(){
        VelocityVoltage velocityRequest = new VelocityVoltage(targetShooterRPS).withSlot(0);
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
      
    public void plusAngle(){
        PositionVoltage hoodBump = new PositionVoltage(getAngle() - 1).withSlot(0);
        hood.setControl(hoodBump);
    }
    
    public void plusVelocity(){
        targetShooterRPS += 5.0; // Adds 5 to the target
        if (targetShooterRPS > 101) { 
            targetShooterRPS = 101; // Max limit cap
        }
        goShoot(); 
    }
    
    public void minusVelocity(){
        targetShooterRPS -= 5.0; // Subtracts 5 from the target
        if (targetShooterRPS < 0) { 
            targetShooterRPS = 0; // Prevents it from going negative!
        }
        goShoot();
    }

    public void stop(){
        hood.set(0);
        targetShooterRPS = 0.0; // Reset target speed
        leadShoot.set(0); // Stop the wheels
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hood Angle", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Target Angle", pos.getAngle());
        SmartDashboard.putNumber("Velocity", leadShoot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Target Velocity", targetShooterRPS); 
        SmartDashboard.putBoolean("Flywheel Ready", isFlywheelReady()); // Added this so you can see it on the dash!
    }
}