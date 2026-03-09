package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

    //Shooter has 2 motors that runs the flywheels, they are opposed - we having one follow the other
    TalonFX leadShoot;
    TalonFX followShoot;

    // We store the target RPS here so the plus/minus buttons work correctly
    private double targetShooterRPS = 0.0; 

    //  Configs for the 2 motors and their PID settings
    public Flywheel(){

        //On the Canbus UPPER
        leadShoot = new TalonFX(27, "Upper");
        followShoot = new TalonFX(15,"Upper");
    
        // --- Shooter Config - COAST mode safer for flywheels
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leadShoot.getConfigurator().apply(flywheelConfig);

        // --- SHOOTER CONFIG ---
        Slot0Configs flywheelPID = new Slot0Configs();
        flywheelPID.kP = 2.0; // Changed to 2.0 for smoother recovery
        flywheelPID.kI = 0.0;
        flywheelPID.kD = 0.0;
        flywheelPID.kV = 0.12;
        flywheelPID.kS = 0.0;
        flywheelPID.kA = 1.0;
        
        leadShoot.getConfigurator().apply(flywheelPID);
        followShoot.getConfigurator().apply(flywheelPID);

        TalonFXConfiguration flywheelfollowConfig = new TalonFXConfiguration();
        flywheelfollowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followShoot.getConfigurator().apply(flywheelfollowConfig);
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));
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
        // 2 RPS is 180 RPM
        return Math.abs(currentRPS - targetShooterRPS) <= 2.0;
    }

    //shoots using the stored targetRPS variable
    public void goShoot(){
        VelocityVoltage velocityRequest = new VelocityVoltage(targetShooterRPS).withSlot(0);
        leadShoot.setControl(velocityRequest);
    }

    public void stop(){
        targetShooterRPS = 0.0; // Reset target speed
        leadShoot.set(0); // Stop the wheels
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Velocity", leadShoot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Target Velocity", targetShooterRPS); 
        SmartDashboard.putBoolean("Flywheel Ready", isFlywheelReady()); // Added this so you can see it on the dash!
    }
    
}
