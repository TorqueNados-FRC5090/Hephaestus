    package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakePosition;

public class Intake extends SubsystemBase{
     TalonFX intakeMotor;
     TalonFX rotationMotor;
     IntakePosition pos;

    /** Constructs an Intake
     *  @param intakeID The ID of the intake motor
     *  @param rotateID The ID of the rotate motor
     */
    public Intake(int intakeID, int rotateID){
        intakeMotor = new TalonFX(intakeID, "Upper");        
        rotationMotor = new TalonFX(rotateID, "Upper");
        Slot0Configs intakePIDConfig = new Slot0Configs();
        intakePIDConfig.kP = 1.2;
        intakePIDConfig.kD = .0;
        intakePIDConfig.kV = 0;
        rotationMotor.getConfigurator().apply(intakePIDConfig);
        intakeMotor.getConfigurator().apply(intakePIDConfig);
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

    // Go-go Gadget Stop (Stops the Intake)
    public void full(){
        intakeMotor.set(0);
    }
    
    // Go-go Gadget Rotate-No-More
    public void rotatestop(){
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
        SmartDashboard.putNumber("Intake Position Degrees", rotationMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake RPM", intakeMotor.getVelocity().getValueAsDouble());
    }

    
}

