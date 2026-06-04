package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.RollerSystem;

public class c3 extends Command {
    Shooter shooter;
    RollerSystem rollersystem;
    double RPS;

     public c3(Shooter shoot, RollerSystem rollersystem, double RPS){
        this.shooter = shoot;
        this.rollersystem = rollersystem;
        this.RPS = RPS;

        addRequirements(shoot, rollersystem);
    }
  
    @Override
    public void initialize(){
        rollersystem.roll(.4);
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
