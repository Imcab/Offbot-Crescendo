package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class intake extends SubsystemBase{
    CANSparkMax intake, index;
    RelativeEncoder enc_intake;

    DigitalInput beam;

    private final SimpleMotorFeedforward FF;
    private final PIDController FeedBackController;

    public intake(){
        
        FeedBackController = new PIDController(IntakeConstants.kp, 0, IntakeConstants.kd);
        FF = new SimpleMotorFeedforward(IntakeConstants.ks, IntakeConstants.kv);

        intake = new CANSparkMax(IntakeConstants.Intakeport, MotorType.kBrushless);
        index = new CANSparkMax(ShooterConstants.OutakeConstants.IndexerPort, MotorType.kBrushless);

        intake.setIdleMode(IdleMode.kBrake);
        index.setIdleMode(IdleMode.kCoast);

        beam = new DigitalInput(4);

        intake.restoreFactoryDefaults();

        intake.setCANTimeout(250);

        intake.setInverted(IntakeConstants.IntakeMotorReversed);
        
        intake.enableVoltageCompensation(12.0);

        enc_intake = intake.getEncoder();

        intake.setCANTimeout(0);

        intake.burnFlash();

    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("InfraRojo", getIR());
    }

    public void setIndex(double speed){
        index.set(speed);
    }
    public void setIntake(double setpointRPM){
        intake.setVoltage(FF.calculate(Units.rotationsPerMinuteToRadiansPerSecond(setpointRPM)) + FeedBackController.calculate(Units.rotationsPerMinuteToRadiansPerSecond(enc_intake.getVelocity()), Units.rotationsPerMinuteToRadiansPerSecond(setpointRPM)));
    }

    public void runIntake(double speed){
        intake.set(speed);
    }

    public void stopIndex(){
        index.set(0);
    }
    public void stopIntake(){
        intake.set(0);
    }
    
    public boolean getIR(){
        return beam.get();
    }


}
