package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.RollerSystem;

public class theYappy extends Command{
    RollerSystem rollers;
    BooleanSupplier runCondition;

    /** Run everything backwards this long when a shot starts, pulling fuel already in the tunnel back down so
     *  the whole column starts moving together (continuous flow). The shooter is still spinning up meanwhile. */
    private static final double kPrimeReverseSeconds = 0.2;
    private final Timer timer = new Timer();

    /** Called once each time feeding starts (used for the shot log) */
    private final Runnable onFeedStart;
    private boolean wasFeeding = false;

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition){
        this(rollers, runCondition, () -> {});
    }

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition, Runnable onFeedStart){
        this.rollers = rollers;
        this.runCondition = runCondition;
        this.onFeedStart = onFeedStart;

        addRequirements(rollers);
    }
    
    @Override
    public void initialize(){
        timer.restart();
        wasFeeding = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < kPrimeReverseSeconds) {
            rollers.roll(RollerSystem.kUnjamSpeedRPS);
            SmartDashboard.putString("Feeder/State", "Priming (backwards)");
        }
        else if (runCondition.getAsBoolean()) {
            if (!wasFeeding) {
                onFeedStart.run();
            }
            wasFeeding = true;
            rollers.roll(RollerSystem.kFeedSpeedRPS); 
            SmartDashboard.putString("Feeder/State", "FEEDING");
        }
        else{ 
            wasFeeding = false;
            rollers.rollerStop();
            SmartDashboard.putString("Feeder/State", "Waiting for Ready (see Ready/ lights)");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       rollers.rollerStop();
       SmartDashboard.putString("Feeder/State", "Off (not shooting)");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}

