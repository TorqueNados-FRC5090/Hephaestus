 
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Spindex;

public class SpindexYappy extends Command{
    Spindex spinner;
    public SpindexYappy(Spindex spinner){
        this.spinner = spinner;
    }
    
    @Override
    public void initialize(){
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
          spinner.spin(-.8);
      //spinner.spin(.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       spinner.spindexStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}

