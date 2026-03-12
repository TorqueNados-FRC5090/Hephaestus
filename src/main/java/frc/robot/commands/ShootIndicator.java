package frc.robot.commands;

import frc.robot.subsystems.CANdleLight;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootIndicator extends Command {
    CANdleLight candle;

    public ShootIndicator(CANdleLight candle){
        this.candle = candle;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        candle.shootindicator();
    }
    
    @Override
    public void end(boolean interrupted){
        candle.shootlightsoff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
