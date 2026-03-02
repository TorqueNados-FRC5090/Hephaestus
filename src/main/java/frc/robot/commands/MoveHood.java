package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class MoveHood extends Command {
    Shooter shoot;
    ShooterPosition target;
     public MoveHood(Shooter shoot, ShooterPosition targetPos){
        this.shoot = shoot;
        this.target = targetPos;

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
        shoot.setTarget(target);
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
      return shoot.atSetpoint();
      
    }
}
