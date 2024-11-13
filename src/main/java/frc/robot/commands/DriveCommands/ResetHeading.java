package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve;

public class ResetHeading extends Command{
    private final swerve drivetrain;

    public ResetHeading(swerve drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        DriverStation.reportWarning("Successfully reset heading", false);
        drivetrain.resetHeading();
    }

    @Override
    public boolean isFinished(){

        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
