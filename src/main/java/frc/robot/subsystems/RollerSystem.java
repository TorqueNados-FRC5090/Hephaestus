package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue; // Restored this!
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSystem extends SubsystemBase {
    /** Feed speed in RPS while shooting. Krakens/Falcons top out around 100 RPS. */
    public static final double kFeedSpeedRPS = 80;
    /** Unjam speed in RPS: floor, lower and upper tunnel all run backwards (the belts follow the floor) */
    public static final double kUnjamSpeedRPS = -40;
    

    TalonFX rollerFloor; 
    TalonFX lowerBelt; 
    TalonFX upperBelt;
    
    public RollerSystem(CANBus canbus){
        rollerFloor = new TalonFX(13, canbus);
        lowerBelt = new TalonFX(21, canbus);
        upperBelt = new TalonFX(22, canbus);

        Slot0Configs rollerConfig = new Slot0Configs();
        rollerConfig.kP = 0.3;
        // Volts per RPS. With only kP the rollers never reached their target speed.
        rollerConfig.kV = 0.5;
      //   rollerConfig.
        rollerFloor.getConfigurator().apply(rollerConfig);
        lowerBelt.getConfigurator().apply(rollerConfig);
        upperBelt.getConfigurator().apply(rollerConfig);

        // Set to Brake mode so they stop instantly when power is cut
        rollerFloor.setNeutralMode(NeutralModeValue.Brake);
        lowerBelt.setNeutralMode(NeutralModeValue.Brake);
        upperBelt.setNeutralMode(NeutralModeValue.Brake);

        // Using the correct CTRE Phoenix 6 (v25+) enum for followers
        lowerBelt.setControl(new Follower(13, MotorAlignmentValue.Opposed));
        upperBelt.setControl(new Follower(13, MotorAlignmentValue.Opposed)); // Change to .Aligned if it spins backwards!
    } 

    // Sets speed using PID
    public void roll(double rollerSpeed){
        VelocityVoltage velocityRequest = new VelocityVoltage(rollerSpeed).withSlot(0).withEnableFOC(true);
        rollerFloor.setControl(velocityRequest);
    }

    // Cuts power to 0 volts instead of fighting PID, stopping the jitter!
    public void rollerStop(){
        rollerFloor.setControl(new VoltageOut(0));
    } 

    public void unjam(){
        VelocityVoltage unjamRequest = new VelocityVoltage(kUnjamSpeedRPS).withSlot(0).withEnableFOC(true);
        rollerFloor.setControl(unjamRequest);
    }

    public Command otherUnjam(){
        return runEnd(
            () -> roll(kUnjamSpeedRPS), 
            () -> rollerStop()
        );
    }

     @Override
    public void periodic() {
         SmartDashboard.putNumber("Roller Floor voltage", rollerFloor.getMotorVoltage().getValueAsDouble());
    }
}