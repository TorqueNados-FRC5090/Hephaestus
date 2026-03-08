package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindex;

public class BumpHood extends Command {
    Shooter shoot;
    ShooterPosition target;
    Spindex spindex;
    //double down;
     public BumpHood(Shooter shoot, Spindex spinner){
        this.shoot = shoot;
        this.spindex = spinner;
       //  down = shoot.getAngle();
        

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
       // down = shoot.getAngle() - 1;
    }

    @Override
    public void execute(){
        
         shoot.plusAngle();
    }

    @Override 
    public void end(boolean interrupted){
    
        
    }

    @Override
    public boolean isFinished(){
      return false;
      
    }
}
