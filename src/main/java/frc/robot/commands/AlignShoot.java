package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.pitch;
import frc.robot.subsystems.shooter;

public class AlignShoot extends Command{

    pitch angle;
    shooter wheels;
    intake index;
    Debouncer timer = new Debouncer(0.7);

    double RPS,speed, degrees;
    public AlignShoot(pitch angle, shooter wheels, intake index, double RPS, double degrees, double speed){
        this.angle = angle;
        this.index = index;
        this.wheels = wheels;
        this.RPS = RPS;
        this.degrees = degrees;
        this.speed = speed;

        addRequirements(angle, wheels);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        angle.runShooterAngle(degrees);
        
        if (angle.getDegrees() >= (degrees - 3)) {
            wheels.setWheels(RPS);
        }
        if (wheels.getRPS() >= (RPS * 0.75)) {
            index.setIndex(speed);
        }

    }
    
    @Override
    public void end(boolean interrupted) {
        angle.stop();
        index.stopIndex();
        wheels.stop();
    }
    
    @Override
    public boolean isFinished(){
        return timer.calculate(wheels.getRPS() >= (RPS * 0.75));
    }
}
