package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class BumpVelocity extends Command {
    Shooter shooter;
    double RPS;

     public BumpVelocity(Shooter shoot, double RPS){
        this.shooter = shoot;
        this.RPS = RPS;

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){
        shooter.incrementVelocityBy(RPS);
    }

    @Override
    public void execute(){
    }

    @Override 
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
        return shooter.isShooterReady(2);
    }
}
