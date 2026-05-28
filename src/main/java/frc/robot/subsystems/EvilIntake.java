// if this doesn't work i explode the robot. Ok?

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     TalonFX intakeMotor;
     TalonFX spinMotor;
     EvilIntakePosition pos = EvilIntakePosition.in;

    /** Constructs an Intake
     *  @param intakeID The ID of the intake motor
     *  @param spinID The ID for the spinning part of the intake
     */
    public EvilIntake(int intakeID, int spinID){
        intakeMotor = new TalonFX(intakeID, "Upper");
        spinMotor = new TalonFX(spinID, "Upper");       
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        /* config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; */
        // yeah idk will do research on if its inverted or not lol
        /* config.Slot0.kP = 72;
        config.Slot0.kD = 0;
        config.Slot0.kV = 0;
        config.Slot0.kG = 1.8;
        config.Slot0.kA = 0; */
        // pdvga......................................................!!
        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinMotor.getConfigurator().apply(spinConfig);
    }
    
    // Go-go Gadget Rotate (Makes Intake Rotate)
    public void evilyummy(EvilIntakePosition pos){
        this.pos = pos;
        PositionVoltage rotationRequest = new PositionVoltage(pos.getAngle()).withSlot(0);
        intakeMotor.setControl(rotationRequest);
    }
    
    public void evileryummy(double speed){
        spinMotor.set(speed);
    }
    
    public double getAngle(){
        return intakeMotor.getRotorPosition().getValueAsDouble();
    }

    
    @Override
    public void periodic() {
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