package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Zero extends Command {
    Shooter shooter;
    double RPS;
    
    public Zero(Shooter shooter, double RPS){
        this.shooter = shooter;
        this.RPS = RPS;
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        shooter.stop();
    }

   @Override 
    public void end(boolean interrupted){
        if(interrupted){
            shooter.stop();
        }
    }

    @Override
    public boolean isFinished(){
        return shooter.isShooterReady();
    }

}
