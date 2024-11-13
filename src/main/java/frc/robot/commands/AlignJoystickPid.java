package frc.robot.commands;

import java.util.function.DoubleSupplier;


import frc.robot.subsystems.pitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignJoystickPid extends Command{

    private final pitch shoterAngle;
    private final DoubleSupplier joystickSupplier;
    double angle;

    public AlignJoystickPid(pitch shooterAngle, DoubleSupplier joystickSupplier){
        this.shoterAngle = shooterAngle;
        this.joystickSupplier = joystickSupplier;
        addRequirements(shooterAngle);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        double joystickValue = joystickSupplier.getAsDouble();
    
        
        if (Math.abs(joystickValue) < 0.5){
            angle = shoterAngle.getDegrees();
            shoterAngle.runShooterAngle(angle);

        }
 
            if(joystickValue > 0.5  && shoterAngle.getDegrees() < 56){
                angle = shoterAngle.getDegrees() + 3 * (50 - shoterAngle.getDegrees()) * joystickValue / 25;
                shoterAngle.runShooterAngle(angle);
                //SmartDashboard.putNumber("setpoint", angle);
                
             }
            else if(joystickValue < -0.5 && shoterAngle.getDegrees() > 3){
                angle =  shoterAngle.getDegrees()  + 2.5 * (shoterAngle.getDegrees()) *  joystickValue / 25;
                shoterAngle.runShooterAngle(angle);
                //SmartDashboard.putNumber("setpoint", angle);   
            }else{
                //shooterAngle.stop();
            }

            SmartDashboard.putNumber("setpoint", angle);

             
    }

    @Override
    public void end(boolean interrupted) {
        shoterAngle.stop();  
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }

}
