 
package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Spindex;

public class SpindexYappy extends Command{
    Spindex spinner;
    BooleanSupplier runCondition;

    public SpindexYappy(Spindex spinner, BooleanSupplier runCondition){
        this.spinner = spinner;
        this.runCondition = runCondition;

        addRequirements(spinner);
    }
    
    @Override
    public void initialize(){
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (runCondition.getAsBoolean())
            spinner.spin(-.8);
        else spinner.spindexStop();
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

