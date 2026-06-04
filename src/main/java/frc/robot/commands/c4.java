package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.EvilIntake;

public class c4 extends Command{
    EvilIntake evilIntake;
    EvilIntakePosition evilTarget;
    public c4(EvilIntake evilIntake, EvilIntakePosition evilPos){
        this.evilIntake = evilIntake;
        this.evilTarget = evilPos;

        addRequirements(evilIntake);
    }
    
    @Override
    public void initialize() {
        evilIntake.evilyummy(evilTarget);
        new WaitCommand(.125);   
        evilIntake.evileryummy(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
     @Override
    public void end(boolean interrupted) {
        evilIntake.evilyummy(EvilIntakePosition.in);
        evilIntake.evileryummy(0);
    } 

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}