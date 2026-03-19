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
    // Shooter has 2 motors that runs the shooters, they are opposed - we have one follow the other
    private final TalonFX leadShoot;
    private final TalonFX followShoot;

    /** We store the target RPS here so the plus/minus buttons work correctly */
    private double setpoint = 0.0; 

    // Configs for the 2 motors and their PID settings
    public Shooter() {
        // On the Canbus UPPER
        leadShoot = new TalonFX(27, "Upper");
        followShoot = new TalonFX(15,"Upper");
    
        // --- LEAD SHOOTER CONFIG ---
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast is safer for heavy flywheels
        
        // PID Configuration
        shooterConfig.Slot0.kP = 0.4; 
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0005;
        shooterConfig.Slot0.kV = 0.16;
        shooterConfig.Slot0.kS = 0.0;
        
        leadShoot.getConfigurator().apply(shooterConfig);

        // --- FOLLOWER CONFIG ---
        TalonFXConfiguration shooterFollowConfig = new TalonFXConfiguration();
        shooterFollowConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followShoot.getConfigurator().apply(shooterFollowConfig);
        
        // Command the follower to perfectly mirror the lead motor, but reversed
        followShoot.setControl(new Follower(leadShoot.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /** Checks if the shooter is up to speed and ready to fire */
    public boolean isShooterReady(double range) {
        // If we aren't trying to shoot, the shooter isn't "ready"
        if (setpoint == 0.0) {
            return false;
        }

        // Get the current speed
        double currentRPS = leadShoot.getVelocity().getValueAsDouble();

        // Check if the current speed is within the given RPS range of our target
        return Math.abs(currentRPS - setpoint) <= range;
    }

    /** Updates the targetRPS and sends it to the shooter */
    public void goShoot(double RPS) {
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

    public void stop() {
        setpoint = 0.0; // Reset target speed
        leadShoot.stopMotor(); // Safely cuts power to the motor rather than commanding 0 RPS closed-loop
    }

    /** * Returns a command that drives the shooter, then stops it at command end.
     * SOTM MAGIC: Because RPS is a DoubleSupplier, it will constantly recalculate
     * the optimal speed based on robot velocity while the button is held down!
     */
    public Command shoot(DoubleSupplier RPS) {
        return this.runEnd(
            () -> goShoot(RPS.getAsDouble()), 
            () -> stop()
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Velocity_RPS", leadShoot.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Target_Velocity_RPS", setpoint); 
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady(2)); 
    }   
}