package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class Shoot extends Command{
    private final shooter wheels;
    private final intake intake;
    Double setpointRPS;

    public Shoot(shooter wheels, intake intake, Double setpointRPS){
        this.wheels = wheels;
        this.intake = intake;
        this.setpointRPS = setpointRPS;
 
        addRequirements(wheels, intake);
    }
   
    @Override
    public void initialize(){}

    @Override
    public void execute(){

        wheels.setWheels(setpointRPS);

        if (wheels.getRPS() >= (setpointRPS * 0.80)) {
            intake.setIndex(1);
        }
        
    }
    @Override
    public void end(boolean interrupted) {
        if(DriverStation.isAutonomousEnabled() == false){
            wheels.stop();
            intake.stopIndex();
        }
        
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
