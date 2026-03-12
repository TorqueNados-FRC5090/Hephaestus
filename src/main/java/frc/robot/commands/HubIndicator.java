package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLight;

public class HubIndicator extends Command {
    CANdleLight candle;

    public HubIndicator(CANdleLight candle){
        this.candle = candle;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        candle.hubindicator();
    }
    
    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}
