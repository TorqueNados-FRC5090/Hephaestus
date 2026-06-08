package frc.robot.subsystems;

import org.ejml.equation.Variable;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     TalonFX intakeMotor;
     TalonFX spinMotor;
     EvilIntakePosition pos = EvilIntakePosition.in;

     // Define the control request once up here to save Garbage Collection overhead!
     final PositionVoltage rotationRequest = new PositionVoltage(0).withSlot(0);

     //debugging
     boolean hitPoint = false;
     double hitPointValue;
     int intakevalueid = 0;

    /** Constructs an Intake
     * @param intakeID The ID of the intake motor
     * @param spinID The ID for the spinning part of the intake
     */
    public EvilIntake(int intakeID, int spinID, CANBus canbus){
        intakeMotor = new TalonFX(intakeID, canbus);
        spinMotor = new TalonFX(spinID, canbus);  
        
        intakevalueid = intakeID;

        // --- INTAKE MOTOR CONFIGURATION ---
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        
        // 1. Set to Coast Mode
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // 2. Limit Torque to 5 Amps so it gives up when hit
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; */
        // yeah idk will do research on if its inverted or not lol
        
        // 3. PID Tuning (Uncommented and fixed variable name to intakeConfig)
        intakeConfig.Slot0.kP = 72; // Note: 72 is high for Phoenix 6, get ready to tune!
        intakeConfig.Slot0.kD = 0;
        intakeConfig.Slot0.kV = 0;
        //intakeConfig.Slot0.kG = 1.8;
        //intakeConfig.Slot0.kA = 0; 
        // pdvga......................................................!!
        
        // Apply configs to the intake motor
        intakeMotor.getConfigurator().apply(intakeConfig);

        // --- SPIN MOTOR CONFIGURATION ---
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinMotor.getConfigurator().apply(spinConfig);
    }

        // Go-go Gadget Move (Makes the Intake Move)
    public void evilyummy(){
        //intakeMotor.set(1);
    }
    
    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){ //pos is 0.35 comming in from file "Constants.java" through "RobotContainer.java" Through "EvilIntakePiece.java" to here.
        hitPoint = true;
        hitPointValue = pos.getAngle();

        //this.pos = pos;
        
        // Update the pre-allocated request instead of making a new one every loop
        intakeMotor.setControl(rotationRequest.withPosition(pos.getAngle()));
    }
    
    public void evileryummy(double speed){
        spinMotor.set(speed);
    }

    public Command evilestyummy(EvilIntakePosition pos){
        return run(() -> evilyummy(pos));
    }
    
    public double getAngle(){
        return intakeMotor.getRotorPosition().getValueAsDouble();
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake ID", intakevalueid);
        SmartDashboard.putBoolean("Intake HitPoint", hitPoint);
        SmartDashboard.putNumber("Intake HitPointValue", hitPointValue);
        SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
        /* SmartDashboard.putNumber("Intake Position Degrees", getAngle());
        SmartDashboard.putString("Intake Target Position", pos.name());
        SmartDashboard.putNumber("Intake Target Revolutions", pos.getAngle());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("stator current", rotationMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("supply current", rotationMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("torque current", rotationMotor.getTorqueCurrent().getValueAsDouble()); */
    }
}