package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class c6 extends Command {
    Hood hood;
    double target;

     public c6(Hood hood, double target){
        this.hood = hood;
        this.target = target;

        addRequirements(hood);
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        hood.goTo(target);
    }/* 
    @Override 
    public void end(boolean interrupted){
        if(interrupted){
            hood.goTo(0);
        }
    }
*/
    @Override
    public boolean isFinished(){
        return false;
    }
}
