package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    Shooter shooter;
    DoubleSupplier RPS;
    
    public Shoot(Shooter shooter, DoubleSupplier RPS){
        this.shooter = shooter;
        this.RPS = RPS;
    }
  
    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        shooter.goShoot(RPS.getAsDouble());
    }

   @Override 
    public void end(boolean interrupted){
        //if(interrupted){
        //    shooter.stop();
        //}
    }

    @Override
    public boolean isFinished(){
        return shooter.isShooterReady();
    }

}
