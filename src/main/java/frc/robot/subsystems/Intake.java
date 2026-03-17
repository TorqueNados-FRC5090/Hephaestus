    package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase {
     TalonFXS intakeMotor;
     TalonFX rotationMotor;
     IntakePosition pos = IntakePosition.up;
     int intakeID = 12;
     int rotateID = 11;

    /** Constructs an Intake
     *  @param intakeID The ID of the intake motor
     *  @param rotateID The ID of the rotate motor
     */
    public Intake(/*int intakeID, int rotateID*/){
        intakeMotor = new TalonFXS(intakeID, "Upper");        
        rotationMotor = new TalonFX(rotateID, "Upper");

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        intakeMotor.getConfigurator().apply(config);

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(20));
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Feedback.SensorToMechanismRatio = 50;
        pivotConfig.Slot0.kP = 72;
        pivotConfig.Slot0.kD = 0;
        pivotConfig.Slot0.kV = 0;
        pivotConfig.Slot0.kG = 1.8;
        pivotConfig.Slot0.kA = 0;
        rotationMotor.getConfigurator().apply(pivotConfig);
     }
    
    // Go-go Gadget Move (Makes the Intake Move)
    public void yummy(){
        intakeMotor.set(-.8);
    }
    
    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void rotate(IntakePosition pos){
        this.pos = pos;
        PositionVoltage rotationRequest = new PositionVoltage(pos.getAngle()).withSlot(0);
        rotationMotor.setControl(rotationRequest);
    }

    // Go-go Gadget Rotate (Makes Intake Rotate)
    public Command rotateCommand(IntakePosition pos){
        return run(() -> rotate(pos));
    }

    // Go-go Gadget Stop (Stops the Intake)
    public void full(){
        intakeMotor.set(0);
    }
    
    // Go-go Gadget Rotate-No-More
    public void rotateStop(){
        rotationMotor.set(0);
    }

    public double getAngle(){
        return rotationMotor.getPosition().getValueAsDouble();

    }

    public boolean setpointCheck(){
        return Math.abs(getAngle() - pos.getAngle()) <= .5;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position Degrees", getAngle());
        SmartDashboard.putString("Intake Target Position", pos.name());
        SmartDashboard.putNumber("Intake Target Revolutions", pos.getAngle());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("stator current", rotationMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("supply current", rotationMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("torque current", rotationMotor.getTorqueCurrent().getValueAsDouble());
    }
}

