package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     TalonFX intakeMotor;
     EvilIntakePosition pos = EvilIntakePosition.out;
     int intakeID = 12;
     int rotateID = 11;

    /** Constructs an Intake
     *  @param intakeID The ID of the intake motor
     *  @param rotateID The ID of the rotate motor
     */
    public EvilIntake(/*int intakeID, int rotateID*/){
        intakeMotor = new TalonFX(intakeID, "Upper");        

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        /* config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; */
        // yeah idk will do research on if its inverted or not lol
        /* config.Slot0.kP = 72;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0;
        config.Slot0.kG = 1.8;
        config.Slot0.kA = 0; */
        // pdvga......................................................!!
        intakeMotor.getConfigurator().apply(config);
    }
    
    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){
        this.pos = pos;
    }
    
    /* @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position Degrees", getAngle());
        SmartDashboard.putString("Intake Target Position", pos.name());
        SmartDashboard.putNumber("Intake Target Revolutions", pos.getAngle());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("stator current", rotationMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("supply current", rotationMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("torque current", rotationMotor.getTorqueCurrent().getValueAsDouble());
    } */
}