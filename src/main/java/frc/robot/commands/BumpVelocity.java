package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindex;

public class BumpVelocity extends Command {
    Shooter shooter;
    Spindex spindex;
    double RPS;

     public BumpVelocity(Shooter shoot, Spindex spindex, double RPS){
        this.shooter = shoot;
        this.spindex = spindex;
        this.RPS = RPS;

        addRequirements(shoot, spindex);
    }
  
    @Override
    public void initialize(){
        spindex.spin(-.4,-.8);
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
