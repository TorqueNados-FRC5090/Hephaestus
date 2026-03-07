package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class BumpHood extends Command {
    Shooter shoot;
    ShooterPosition target;
    double angle;
     public BumpHood(Shooter shoot, double angle){
        this.shoot = shoot;
        this.angle = angle;

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
      angle = angle - 1;
    }

    @Override
    public void execute(){
     shoot.plusAngle(angle);
    }

    @Override 
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
      return false;
      
    }
}
