package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;

public class MoveHood extends Command {
    Hood hood;
    DoubleSupplier target;

     public MoveHood(Hood hood, DoubleSupplier target){
        this.hood = hood;
        this.target = target;

        addRequirements(hood);
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        hood.goTo(target.getAsDouble());
    }

    @Override 
    public void end(boolean interrupted){
        if(interrupted){
            hood.goTo(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
