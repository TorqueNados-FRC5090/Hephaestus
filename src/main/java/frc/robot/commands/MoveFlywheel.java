package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class MoveFlywheel extends Command {
    Flywheel flywheel;
    
    public MoveFlywheel(Flywheel flywheel){
        this.flywheel = flywheel;
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        flywheel.goShoot(50);
    }

   @Override 
    public void end(boolean interrupted){
        flywheel.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}