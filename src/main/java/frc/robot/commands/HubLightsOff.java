package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLight;

public class HubLightsOff extends Command {
    CANdleLight candle;

    public HubLightsOff(CANdleLight candle){
        this.candle = candle;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        candle.hublightsoff();
    }
    
    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
