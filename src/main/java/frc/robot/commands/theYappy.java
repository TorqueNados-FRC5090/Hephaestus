package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.RollerSystem;

public class theYappy extends Command{
    RollerSystem rollers;
    BooleanSupplier runCondition;

    /** Run everything backwards this long when a shot starts, pulling fuel already in the tunnel back down so
     *  the whole column starts moving together (continuous flow). The shooter is still spinning up meanwhile. */
    private static final double kPrimeReverseSeconds = 0.2;
    private final Timer timer = new Timer();

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition){
        this.rollers = rollers;
        this.runCondition = runCondition;

        addRequirements(rollers);
    }
    
    @Override
    public void initialize(){
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < kPrimeReverseSeconds) {
            rollers.roll(RollerSystem.kUnjamSpeedRPS);
        }
        else if (runCondition.getAsBoolean()) {
            rollers.roll(RollerSystem.kFeedSpeedRPS); 
        }
        else{ 
            rollers.rollerStop();
        }
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

