package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindex;

public class VeloDown extends Command {
    Shooter shoot;
    ShooterPosition target;
    Spindex spindex;
     public VeloDown(Shooter shoot, Spindex spindex){
        this.shoot = shoot;
        this.spindex = spindex;

        addRequirements(shoot);
    }
  
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        spindex.spin(-.8);
        shoot.minusVelocity();
    }

    @Override 
    public void end(boolean interrupted){
    
    }

    @Override
    public boolean isFinished(){
      return false;
      
    }
}
