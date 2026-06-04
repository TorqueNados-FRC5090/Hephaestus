package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.EvilIntakePosition;
import frc.robot.subsystems.EvilIntake;

public class EvilIntakeToggle extends Command{
    EvilIntake evilIntake;
    EvilIntakePosition evilTarget;
    boolean OutNow = true;

        public EvilIntakeToggle(EvilIntake evilIntake){
            this.evilIntake = evilIntake;

            addRequirements(evilIntake);
        }
        
        @Override
        public void initialize() {
            if (OutNow = false){
                evilIntake.evilyummy(EvilIntakePosition.out);
                new WaitCommand(.125);   
                evilIntake.evileryummy(1);
                OutNow = true;
            }
            else {
                evilIntake.evileryummy(0);
                evilIntake.evilyummy(EvilIntakePosition.in);
                OutNow = false;
            }
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted){} 

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false; // Has no end condition
        }
}