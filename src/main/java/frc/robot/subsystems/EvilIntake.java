package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.EvilIntakePosition;

public class EvilIntake extends SubsystemBase {
     TalonFX intakeMotor;
     TalonFX spinMotor;
     EvilIntakePosition pos = EvilIntakePosition.in;

     // --- AUTO AGITATE (rack and pinion) ---
     /** Roller power while intaking. Negative = pulls fuel in (same as EvilIntakePiece). */
     private static final double kRollerIntakeSpeed = -1.0;
     /** Roller power while ejecting (spits fuel back out the intake) */
     private static final double kRollerEjectSpeed = 1.0;
     /** How far in the rack slides during agitate, in motor rotations (out = 17, fully in = 0.36).
      *  Only part way: pulling it all the way in with fuel inside jams or breaks it. */
     private static final double kAgitateInRotations = 10.0;
     /** Seconds for one full out -> in -> out cycle. Bigger = slower */
     private static final double kAgitatePeriodSeconds = 2.0;

     // Define the control request once up here to save Garbage Collection overhead!
     final PositionVoltage rotationRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
     final DutyCycleOut spinRequest = new DutyCycleOut(0).withEnableFOC(true);

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

        // 2. Limit Torque to 40 Amps so it gives up when hit
        intakeConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 25;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        /* intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; */
        // yeah idk will do research on if its inverted or not lol
        
        // 3. PID Tuning (Uncommented and fixed variable name to intakeConfig)
        intakeConfig.Slot0.kP = 3; 
       // intakeConfig.Slot0.kD = 0;
       // intakeConfig.Slot0.kV = 1;
       // intakeConfig.Slot0.kG = 1.8;
        // Volts to overcome friction. Was 15, which did nothing with the default sign setting
        // (and would slam full power with the one below). UseClosedLoopSign makes it push toward the setpoint.
        intakeConfig.Slot0.kS = 0.3; 
        intakeConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        // pdvga......................................................!!
        
        // Apply configs to the intake motor
        intakeMotor.getConfigurator().apply(intakeConfig);

        // --- SPIN MOTOR CONFIGURATION ---
        TalonFXConfiguration spinConfig = new TalonFXConfiguration();
        spinConfig.CurrentLimits.StatorCurrentLimit = 40;
        spinConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spinConfig.CurrentLimits.SupplyCurrentLimit = 30;
        spinConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        spinMotor.setControl(spinRequest.withOutput(speed));
    }

    public Command evilestyummy(EvilIntakePosition pos){
        return run(() -> evilyummy(pos));
    }
    
    /** Rollers on while the rack slowly slides in and out, squeezing the hopper so fuel keeps
     *  flowing into the feeder while we shoot. Leaves the rack out and rollers off when it ends. */
    public Command agitate() {
        Timer timer = new Timer();
        double out = EvilIntakePosition.out.getAngle();
        return startRun(
            timer::restart,
            () -> {
                // 0 -> 1 -> 0 over one period, starting from fully out
                double inAmount = (1 - Math.cos(2 * Math.PI * timer.get() / kAgitatePeriodSeconds)) / 2;
                intakeMotor.setControl(rotationRequest.withPosition(out + (kAgitateInRotations - out) * inAmount));
                spinMotor.setControl(spinRequest.withOutput(kRollerIntakeSpeed));
            })
            .finallyDo(() -> {
                intakeMotor.setControl(rotationRequest.withPosition(out));
                spinMotor.setControl(spinRequest.withOutput(0));
            });
    }

    /** Intake out with the wheels spinning backwards to spit fuel out. Pulls back in and stops when released,
     *  same as the intake button. */
    public Command eject() {
        return run(() -> {
            evilyummy(EvilIntakePosition.out);
            spinMotor.setControl(spinRequest.withOutput(kRollerEjectSpeed));
        }).finallyDo(() -> {
            evilyummy(EvilIntakePosition.in);
            spinMotor.setControl(spinRequest.withOutput(0));
        });
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