package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class BumpHood extends Command {
    Shooter shoot;
    ShooterPosition target;
    double angle;
     public BumpHood(Shooter shoot){
        this.shoot = shoot;
        

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
        shoot.plusAngle();
    }

    @Override
    public void execute(){
     
    }

    @Override 
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
      return true;
      
    }
}
