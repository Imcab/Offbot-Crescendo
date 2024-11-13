package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret;

public class AlignTurret extends Command{

    private final turret turret;
    private final Double setpoint;
    boolean isFinished;

    public AlignTurret(turret turret, Double setpoint){

        this.turret = turret;
        this.setpoint = setpoint;
        addRequirements(turret);
    }
 
    @Override
    public void initialize(){}
    @Override
    public void execute(){

        if (turret.getLaps() < -1.2) {
            turret.runTurret(setpoint);
        }else{
            turret.runTurret(setpoint);
        }
                 
    }
    
    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

     @Override
    public boolean isFinished(){

        return false;
    }
    
}