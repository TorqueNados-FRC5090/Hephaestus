package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class c2 extends Command {
    Hood hood;
    double revolutions;

    /** Increments the hood's current setpoint position by n revolutions. 
     *  Provide a negative number to decrement */
    public c2(Hood hood, double revolutions){
        this.hood = hood;
        this.revolutions = revolutions;

        addRequirements(hood);
    }
  
    @Override
    public void initialize(){
        hood.incrementPositionBy(revolutions);
    }

    @Override
    public void execute(){

    }

    @Override 
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
      return hood.atSetpoint();
    }
}
