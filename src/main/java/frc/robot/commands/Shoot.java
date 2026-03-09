package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    double RPS;
    
    public Shoot(Shooter shooter, double RPS){
        this.shooter = shooter;
        this.RPS = RPS;
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        shooter.goShoot(RPS);
    }

   @Override 
    public void end(boolean interrupted){
        shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
