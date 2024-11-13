package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants.AngleShooterConstants;

public class pitch extends SubsystemBase{
    private final TalonFXConfiguration config;
    private final PIDController pid;

    
    TalonFX pitch;
    SparkAbsoluteEncoder enc;
    RelativeEncoder enc_turret;

    public double ShooterPosition = 0.0;

    private final DutyCycleEncoder encoderPitch;
    double offset = Units.degreesToRotations(AngleShooterConstants.offset);
    
    public pitch(){

        encoderPitch = new DutyCycleEncoder(AngleShooterConstants.EncPort);

        pid = new PIDController(AngleShooterConstants.angleConfig.getP(), AngleShooterConstants.angleConfig.getI(), AngleShooterConstants.angleConfig.getD());
        
        pitch = new TalonFX(12);

        config = new TalonFXConfiguration();

        configureKRAKEN();

        pitch.setInverted(AngleShooterConstants.AngleMotorReversed);

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Angulador", getDegrees());
    }

    public void configureKRAKEN(){

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 30;

        pitch.setInverted(AngleShooterConstants.AngleMotorReversed);

        pitch.getConfigurator().apply(config);

    }

    public double getDegrees(){
        return -Units.rotationsToDegrees(encoderPitch.getAbsolutePosition() - offset);
    }
    public void runShooterAngle(double angle){
        pitch.set(pid.calculate(getDegrees(), angle));
    }
    public void runPitchSpeed(double speed){
        pitch.set(speed);
    }
    public void stop(){
        pitch.stopMotor();
    }

}
