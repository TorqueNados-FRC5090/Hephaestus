package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.RollerSystem;

public class theYappy extends Command{
    RollerSystem rollers;
    BooleanSupplier runCondition;

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition){
        this.rollers = rollers;
        this.runCondition = runCondition;

        addRequirements(rollers);
    }
    
    @Override
    public void initialize(){
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (runCondition.getAsBoolean())
            rollers.roll(30);
        else rollers.rollerStop();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       rollers.rollerStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}

