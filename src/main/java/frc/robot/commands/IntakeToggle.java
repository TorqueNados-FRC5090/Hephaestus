package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class IntakeToggle extends Command{
    Intake intake;
    IntakePosition target;
    boolean down = true;

        public IntakeToggle(Intake intake){
            this.intake = intake;

            addRequirements(intake);
        }
        
        @Override
        public void initialize() {
            if (down = false){
                intake.rotate(IntakePosition.out);
                new WaitCommand(.125);   
                intake.yummy();
                down = true;
            }
            else {
                intake.full();
                intake.rotate(IntakePosition.stow);
                down = false;
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
