package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleLight extends SubsystemBase {
    CANdle candle;
    boolean shoottime;
    LarsonAnimation time2shoot;
    FireAnimation time2hub;
    LarsonAnimation shootoff;
    LarsonAnimation huboff;
    public CANdleLight(){
        candle = new CANdle(999);
        shoottime = false;
        time2shoot = new LarsonAnimation(0, 3);
        time2hub = new FireAnimation(4, 7);
        shootoff = new LarsonAnimation(0, 3);
        huboff = new LarsonAnimation(4, 7);
        CANdleConfiguration candleconfig = new CANdleConfiguration();
    }

    public void shootindicator(){
        candle.setControl(time2shoot);
        shoottime = true;
    }

    public void hubindicator(){
        candle.setControl(time2hub);
    }
    
    public void shootlightsoff(){
        candle.setControl(shootoff);
    }

    public void hublightsoff(){
        candle.setControl(huboff);
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            hublightsoff();
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            hubindicator();
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            hublightsoff();
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            hubindicator();
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            hubindicator();
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            hubindicator();
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            if (shift1Active){
                hubindicator();
            }
            else if (!shift1Active){
                hublightsoff();
            }
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            if (!shift1Active){
                hubindicator();
            }
            else if (shift1Active){
                hublightsoff();
            }
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            if (shift1Active){
                hubindicator();
            }
            else if (!shift1Active){
                hublightsoff();
            }
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            if (!shift1Active){
                hubindicator();
            }
            else if (shift1Active){
                hublightsoff();
            }
            return !shift1Active;
        } else {
            // End game, hub always active.
            hubindicator();
            return true;
        }
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("lined up?", shoottime);
        SmartDashboard.putBoolean("is our hub active?", isHubActive());
    };
}
