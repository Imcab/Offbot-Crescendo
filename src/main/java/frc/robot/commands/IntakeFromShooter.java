package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.turret;
import frc.robot.subsystems.pitch;

public class IntakeFromShooter extends Command{
    shooter wheels;
    intake indexer;
    pitch pitch;
    turret turret;
    double angleT;

    public IntakeFromShooter(shooter wheels, intake indexer, turret turret, pitch pitch, double angleT){
        this.wheels = wheels;
        this.indexer = indexer;
        this.pitch = pitch;
        this.turret = turret;

        addRequirements(wheels, indexer, pitch, turret);
    }
   
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        // .|.  

            pitch.runShooterAngle(37);

            if (pitch.getDegrees() >= 33){
                wheels.runSpeed(-0.15);
                indexer.setIndex(-0.1);
            }
        }
        
    

    @Override
    public void end(boolean interrupted) {
        pitch.stop();
        wheels.stop();
        indexer.stopIndex();
    }

    @Override
    public boolean isFinished(){   
        return false;    
    }
}
