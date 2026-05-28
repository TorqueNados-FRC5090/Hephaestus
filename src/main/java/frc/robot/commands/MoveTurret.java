package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class MoveTurret extends Command {
    Turret turret;
    
    public MoveTurret(Turret turret){
        this.turret = turret;

        addRequirements(turret);
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        turret.alignToHub();
    }

   @Override 
    public void end(boolean interrupted){
        if(interrupted){
            turret.goToZero();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
