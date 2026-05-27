package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EvilIntake;
import frc.robot.Constants.IntakeConstants.EvilIntakePosition;

public class ok extends Command {
    EvilIntake evilintake;
    
    public ok(EvilIntake evilintake){
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
