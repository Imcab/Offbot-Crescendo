package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.turret;

public class JoystickTurret extends Command{

    private final turret turret;
    private final DoubleSupplier supplier;
    private final CommandXboxController controller;

    public JoystickTurret(turret turret, DoubleSupplier supplier, CommandXboxController controller){
        this.turret = turret;
        this.supplier = supplier;
        this.controller = controller;

        addRequirements(turret);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){

            double joystickValue = supplier.getAsDouble();
            
            if (Math.abs(joystickValue) < 0.05){
                joystickValue = 0;
            }
            
            if (Math.abs(turret.getLaps()) > 0.78 && !turret.Turretlimit()) {
                controller.getHID().setRumble(RumbleType.kBothRumble, 0.1);
                turret.runSpeed(joystickValue * 0.15);
            }else if (turret.Turretlimit()) {
                controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                turret.runSpeed(joystickValue * 0.05);
            }else{
                controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                turret.runSpeed(joystickValue * 0.2);
            }

    }
    @Override
    public void end(boolean interrupted) {
        turret.stop();
        controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }       
}
