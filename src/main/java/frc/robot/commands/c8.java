package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EvilIntake;
import frc.robot.Constants.EvilIntakePosition;

public class c8 extends Command {
    EvilIntake evilintake;
    
    public c8(EvilIntake evilintake){
        this.evilintake = evilintake;
    }
  
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        evilintake.evilyummy(EvilIntakePosition.out);
        evilintake.evileryummy(1);
    }

   @Override 
    public void end(boolean interrupted){
        evilintake.evilyummy(EvilIntakePosition.in);
        evilintake.evileryummy(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
