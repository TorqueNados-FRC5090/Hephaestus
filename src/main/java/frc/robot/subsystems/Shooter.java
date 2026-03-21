package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    //Shooter has 2 motors that runs the shooters, they are opposed - we having one follow the other
    TalonFX leadShoot;
    TalonFX followShoot;

    /** We store the target RPS here so the plus/minus buttons work correctly */
    private double setpoint = 0.0; 

    //  Configs for the 2 motors and their PID settings
    public Shooter(){

        //On the Canbus UPPER
        leadShoot = new TalonFX(27, "Upper");
        followShoot = new TalonFX(15,"Upper");
    
        // --- Shooter Config - COAST mode safer for shooters
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leadShoot.getConfigurator().apply(shooterConfig);

        // --- SHOOTER CONFIG ---
        Slot0Configs shooterPID = new Slot0Configs();
        shooterPID.kP = .8; // Changed to 2.0 for smoother recovery
        shooterPID.kI = 0.0;
        shooterPID.kD = 0; //.0005
        shooterPID.kV = 0.145; // .16
        shooterPID.kS = 0;
        //shooterPID.kA = 1.0;
        
        leadShoot.getConfigurator().apply(shooterPID);
        //followShoot.getConfigurator().apply(shooterPID);

        TalonFXConfiguration shooterfollowConfig = new TalonFXConfiguration();
        shooterfollowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followShoot.getConfigurator().apply(shooterfollowConfig);
        followShoot.setControl(new Follower(27, MotorAlignmentValue.Opposed));
    }

    /** Checks if the shooter is up to speed and ready to fire */
    public boolean isShooterReady(double range){
        // If we aren't trying to shoot, the shooter isn't "ready"
        if (setpoint == 0.0) {
            return false;
        }

        // Get the current speed
        double currentRPS = leadShoot.getVelocity().getValueAsDouble();

        // Check if the current speed is within 2 RPS of our target
        // 2 RPS is 120 RPM
        return Math.abs(currentRPS - setpoint) <= range;
    }

    /** Updates the targetRPS and sends it to the shooter */
    public void goShoot(double RPS){
        this.setpoint = RPS;
        VelocityVoltage velocityRequest = new VelocityVoltage(setpoint).withSlot(0);
        leadShoot.setControl(velocityRequest);
    }

    /** Adds the given RPS to the shooter's current setpoint. */
    public void incrementVelocityBy(double RPS) {
        this.setpoint += RPS;
        VelocityVoltage velocityRequest = new VelocityVoltage(setpoint).withSlot(0);
        leadShoot.setControl(velocityRequest);
    }

    public void stop(){
        setpoint = 0.0; // Reset target speed
        leadShoot.set(0); // Stop the wheels
    }

    /** Returns a command that drives the shooter, then stops it at command end */
    public Command shoot(DoubleSupplier RPS) {
        return this.runEnd(
            () -> goShoot(RPS.getAsDouble()),
            () -> stop()
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Velocity", leadShoot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Target Velocity", setpoint); 
        SmartDashboard.putBoolean("shooter Ready", isShooterReady(2)); // Added this so you can see it on the dash!
    }   
}
