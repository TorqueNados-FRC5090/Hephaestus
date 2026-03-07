package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class BumpVelocity extends Command {
    Shooter shoot;
    ShooterPosition target;
     public BumpVelocity(Shooter shoot){
        this.shoot = shoot;

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
        shoot.plusVelocity();
    }

    @Override
    public void execute(){
    
    }

    //@Override 
    //public void end(boolean interrupted){
    //    shoot.stop();
    //}

    @Override
    public boolean isFinished(){
      return false;
      
    }
}
