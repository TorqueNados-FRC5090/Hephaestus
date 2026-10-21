package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSystem extends SubsystemBase {
    /** Feed speed in RPS while shooting. Krakens/Falcons top out around 100 RPS. */
    public static final double kFeedSpeedRPS = 90;
    /** Unjam speed in RPS: floor, lower and upper tunnel all run backwards */
    public static final double kUnjamSpeedRPS = -40;
    /** Floor speed while intaking (LB). The tunnels stay off: they only run when shooting. */
    public static final double kIntakeFloorSpeedRPS = 100;

    private final VelocityVoltage m_floorRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final VelocityVoltage m_beltRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut m_stopRequest = new VoltageOut(0).withEnableFOC(true);
    

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

        // Current limits on all three motors (each one gets its own; limits are never shared).
        // Stator 60 A: enough push to keep a full hopper moving and clear light jams; caps heat and belt load
        //   when fuel really jams (the team raised this from 40).
        // Supply 50 A: battery draw per motor; these run at full power while shooting, together with the
        //   shooter, so this keeps the feed from browning the robot out. Lower toward 40 if brownouts return.
        CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true);
        // Floor gets more: it pushes the whole hopper of fuel and wasn't moving it at 60/50 A.
        // Stator 80 A = more push into a full hopper; supply 60 A. Lower again if brownouts come back.
        rollerFloor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true));
        lowerBelt.getConfigurator().apply(rollerLimits);
        upperBelt.getConfigurator().apply(rollerLimits);

        // Set to Brake mode so they stop instantly when power is cut
        rollerFloor.setNeutralMode(NeutralModeValue.Brake);
        lowerBelt.setNeutralMode(NeutralModeValue.Brake);
        upperBelt.setNeutralMode(NeutralModeValue.Brake);

        // The tunnel belts used to be Followers of the floor (Opposed), so they ran whenever the floor ran, including
        // while intaking. Now each motor is commanded on its own. Belts get the opposite sign of the floor, same as
        // the old Opposed follower. If a belt spins the wrong way, flip kBeltDirection.
    } 

    /** -1 = belts spin opposite the floor motor (what the old Opposed followers did) */
    private static final double kBeltDirection = -1.0;

    /** Floor + lower + upper tunnel at this speed (shooting, unjam, eject) */
    public void roll(double rollerSpeed){
        rollerFloor.setControl(m_floorRequest.withVelocity(rollerSpeed));
        lowerBelt.setControl(m_beltRequest.withVelocity(rollerSpeed * kBeltDirection));
        upperBelt.setControl(m_beltRequest.withVelocity(rollerSpeed * kBeltDirection));
    }

    /** Floor only, tunnels off (intaking) */
    public void rollFloorOnly(double rollerSpeed){
        rollerFloor.setControl(m_floorRequest.withVelocity(rollerSpeed));
        lowerBelt.setControl(m_stopRequest);
        upperBelt.setControl(m_stopRequest);
    }

    // Cuts power to 0 volts instead of fighting PID, stopping the jitter!
    public void rollerStop(){
        rollerFloor.setControl(m_stopRequest);
        lowerBelt.setControl(m_stopRequest);
        upperBelt.setControl(m_stopRequest);
    } 

    public void unjam(){
        roll(kUnjamSpeedRPS);
    }

    /** Floor spins to pull fuel in, tunnels stay off. Stops everything when released. */
    public Command intakeFloor(){
        return runEnd(
            () -> rollFloorOnly(kIntakeFloorSpeedRPS),
            () -> rollerStop()
        );
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
         SmartDashboard.putNumber("Roller Lower tunnel voltage", lowerBelt.getMotorVoltage().getValueAsDouble());
         SmartDashboard.putNumber("Roller Upper tunnel voltage", upperBelt.getMotorVoltage().getValueAsDouble());
    }
}