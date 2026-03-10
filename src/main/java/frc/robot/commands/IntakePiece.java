package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class IntakePiece extends Command{
 Intake intake;
 IntakePosition target;
    public IntakePiece(Intake intake, IntakePosition pos){
        this.intake = intake;
        this.target = pos;
    }
    
    @Override
    public void initialize() {
        intake.yummy();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    intake.rotate(target);
    }

    // Called once the command ends or is interrupted.
     @Override
    public void end(boolean interrupted) {
    intake.rotate(IntakePosition.zero);
     intake.full();
    } 

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}
