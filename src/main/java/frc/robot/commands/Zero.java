package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Zero extends Command {
    Shooter shooter;
    Hood hood;
    Turret turret;
    
    public Zero(Shooter shooter, Hood hood, Turret turret){
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;

        addRequirements(shooter, hood, turret);
    }
  
    @Override
    public void initialize(){
        hood.goTo(0);
        turret.goToZero();
        shooter.stop();
    }

    @Override
    public void execute(){
    }

   @Override 
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
