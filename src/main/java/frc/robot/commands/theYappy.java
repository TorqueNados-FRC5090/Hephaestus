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
    /** true = keep the floor pulling fuel in the whole time (shoot on the move), regardless of what phase
     *  the tunnels are in below. false = floor follows the tunnels exactly like before (just shoot). */
    private final boolean alsoIntakeFloor;

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition){
        this(rollers, runCondition, () -> {});
    }

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition, Runnable onFeedStart){
        this(rollers, runCondition, onFeedStart, false);
    }

    public theYappy(RollerSystem rollers, BooleanSupplier runCondition, Runnable onFeedStart,
                     boolean alsoIntakeFloor){
        this.rollers = rollers;
        this.runCondition = runCondition;
        this.onFeedStart = onFeedStart;
        this.alsoIntakeFloor = alsoIntakeFloor;

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
        String suffix = alsoIntakeFloor ? " (+ intaking)" : "";
        if (timer.get() < kPrimeReverseSeconds) {
            if (alsoIntakeFloor) {
                rollers.rollFloorAndBelts(RollerSystem.kIntakeFloorSpeedRPS, RollerSystem.kUnjamSpeedRPS);
            } else {
                rollers.roll(RollerSystem.kUnjamSpeedRPS);
            }
            SmartDashboard.putString("Feeder/State", "Priming (backwards)" + suffix);
        }
        else if (runCondition.getAsBoolean()) {
            if (!wasFeeding) {
                onFeedStart.run();
            }
            wasFeeding = true;
            if (alsoIntakeFloor) {
                rollers.rollFloorAndBelts(RollerSystem.kIntakeFloorSpeedRPS, RollerSystem.kFeedSpeedRPS);
            } else {
                rollers.roll(RollerSystem.kFeedSpeedRPS);
            }
            SmartDashboard.putString("Feeder/State", "FEEDING" + suffix);
        }
        else{ 
            wasFeeding = false;
            if (alsoIntakeFloor) {
                rollers.rollFloorOnly(RollerSystem.kIntakeFloorSpeedRPS);   // keep collecting; tunnels truly off (no jitter)
            } else {
                rollers.rollerStop();
            }
            SmartDashboard.putString("Feeder/State", "Waiting for Ready (see Ready/ lights)" + suffix);
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

