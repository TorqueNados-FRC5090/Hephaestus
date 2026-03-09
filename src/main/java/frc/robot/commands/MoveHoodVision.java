package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants.HoodPosition;
import frc.robot.subsystems.Hood;

public class MoveHoodVision extends Command {
    Hood hood;
    HoodPosition target;
     public MoveHoodVision(Hood hood, HoodPosition targetPos){
        this.hood = hood;
        this.target = targetPos;

        addRequirements(hood);
    }
  
    @Override
    public void initialize(){
        hood.setTarget(target);
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
      return hood.atSetpoint();
      
    }
}
