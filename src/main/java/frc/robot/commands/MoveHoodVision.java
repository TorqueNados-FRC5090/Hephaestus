package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class MoveHoodVision extends Command {
    Hood hood;
     public MoveHoodVision(Hood hood){
        this.hood = hood;
    }
  
    @Override
    public void initialize(){
        hood.setTarget(2);
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
