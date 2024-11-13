package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.turret;

public class feed extends Command{
    intake intake;
    turret turret;
    double speedIndex, speedIntake;
    boolean reverse;
    
    public feed(intake intake, turret turret,double speedIndex, double speedIntake, boolean reverse){
        this.intake = intake;
        this.turret = turret;
        this.speedIndex = speedIndex;
        this.speedIntake = speedIntake;
        this.reverse = reverse;

        addRequirements(intake, turret);
    }

    @Override
    public void execute(){
        turret.runTurret(0.0); 
        intake.setIndex(speedIndex);
        intake.runIntake(speedIntake);
    }

    @Override
    public void end(boolean interrupted){
        intake.stopIndex();
        intake.stopIntake();
    }

    @Override
    public boolean isFinished(){

        return false;
 
    }

}
