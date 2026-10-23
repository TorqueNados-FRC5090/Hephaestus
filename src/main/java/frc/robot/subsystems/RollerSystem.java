package frc.robot.subsystems;

import frc.robot.RobotLog;
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
    private final edu.wpi.first.wpilibj.Timer m_floorStallTimer = new edu.wpi.first.wpilibj.Timer();
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
        // Stator unchanged (that's the push to keep a full hopper moving / clear jams, not what browns us out).
        // Supply 50->40 A (10/22): brownouts came back hard while shooting on the move -- every shot runs the
        // shooter + hood + turret + these rollers + the intake agitating all at once, so the rollers' share of
        // the battery budget is the first thing to trim since it costs the least (shot RPM doesn't depend on
        // these). Raise back toward 50 if the floor/belts feel starved once the battery itself is healthy.
        CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);
        // Floor gets more stator (it pushes the whole hopper of fuel). 80->95 A (10/22 later): the 223626 log
        // shows it stalling ~15 times in under 3 minutes, and every time, voltage was only ~2 V while already AT
        // the stator limit -- that low a voltage at the current cap means it's hitting the limit at basically
        // zero RPM (stall), not fighting the load at speed. That's genuinely not enough torque to get moving
        // again once something backs up against it. Supply stays at 45: the low voltage/duty cycle at stall
        // means supply current is nowhere near its limit here, so that number isn't what's holding it back.
        // If it keeps stalling even at 95, that's a physical obstruction, not a current problem -- raising this
        // forever just means hitting whatever it's stuck on harder.
        rollerFloor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(95)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(45)
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

    /** Floor and tunnels at independent speeds (shoot-on-the-move while still intaking: the floor keeps
     *  pulling fuel in at its own speed while the tunnels do whatever theYappy's feed cycle needs). */
    public void rollFloorAndBelts(double floorSpeed, double beltSpeed){
        rollerFloor.setControl(m_floorRequest.withVelocity(floorSpeed));
        lowerBelt.setControl(m_beltRequest.withVelocity(beltSpeed * kBeltDirection));
        upperBelt.setControl(m_beltRequest.withVelocity(beltSpeed * kBeltDirection));
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
        // Jam warning: floor pushing at its current limit but not turning. In the 10/21 log the floor sat at 0 rot/s
        // with 80 A every time it was commanded (both directions) = mechanically locked, and it sagged the battery.
        boolean floorStalled = Math.abs(rollerFloor.getStatorCurrent().getValueAsDouble()) > 60
            && Math.abs(rollerFloor.getVelocity().getValueAsDouble()) < 1.0;
        if (floorStalled) {
            m_floorStallTimer.start();
        } else {
            m_floorStallTimer.stop();
            m_floorStallTimer.reset();
        }
        boolean floorJammed = m_floorStallTimer.get() > 0.5;
        RobotLog.eventOnRise("floorJam", floorJammed, "FLOOR ROLLER JAMMED: at its current limit but not turning");
        boolean tel = RobotLog.telemetryTick();   // dashboard values at 10 Hz (loop time)

         if (tel) SmartDashboard.putNumber("Roller Floor voltage", rollerFloor.getMotorVoltage().getValueAsDouble());
         if (tel) SmartDashboard.putNumber("Roller Lower tunnel voltage", lowerBelt.getMotorVoltage().getValueAsDouble());
         if (tel) SmartDashboard.putNumber("Roller Upper tunnel voltage", upperBelt.getMotorVoltage().getValueAsDouble());
        if (tel) SmartDashboard.putBoolean("Floor/Jammed", floorJammed);
    }
}