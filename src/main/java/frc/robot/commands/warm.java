package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;
import frc.robot.util.Field3472;

public class warm extends Command{
    shooter wheels;
    double speed;
    public warm(shooter wheels, double speed){
        this.wheels = wheels;
        this.speed = speed;
        addRequirements(wheels);
    }
    @Override
    public void initialize(){}

    @Override
    public void execute(){

        if (Field3472.getZone() == 1) {
           wheels.runSpeed(speed); 
        }else if (Field3472.getZone() == 2) {
            wheels.runSpeed(speed * 0.70);
        }else if (Field3472.getZone() == 3) {
            wheels.runSpeed(speed * 0.5); 
        }else if (Field3472.getZone() == 4) {
            wheels.runSpeed(speed * 0.1);
        }
             
    }
    @Override
    public void end(boolean interrupted) {
        wheels.stop();
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
