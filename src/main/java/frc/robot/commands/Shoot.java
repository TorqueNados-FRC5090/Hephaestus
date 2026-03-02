package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    
    public Shoot(Shooter shooter){
        this.shooter = shooter;
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        shooter.goShoot();
    }

   @Override 
    public void end(boolean interrupted){
   shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
