package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.OutakeConstants;
import frc.robot.util.Field3472;

public class shooter extends SubsystemBase{
    private final TalonFX right;
    private final TalonFX left;
    private final TalonFXConfiguration Shooter;

    private final MotionMagicVelocityVoltage motionMagic;

    public shooter(){
        right = new TalonFX(OutakeConstants.RightWHeelsPort);
        left = new TalonFX(OutakeConstants.LeftWHeelsPort);

        right.setInverted(OutakeConstants.RightMotorReversed);
        left.setInverted(OutakeConstants.LeftMotorReversed);
 
        Shooter = new TalonFXConfiguration();

        motionMagic = new MotionMagicVelocityVoltage(0);

        configureKRAKEN();
    }

    public void configureKRAKEN(){

        Shooter.Slot0.kP = OutakeConstants.wheelsConfig.getP();
        Shooter.Slot0.kI = OutakeConstants.wheelsConfig.getI();
        Shooter.Slot0.kD = OutakeConstants.wheelsConfig.getD();
        Shooter.Slot0.kS = OutakeConstants.wheelsConfig.getS();
        Shooter.Slot0.kV = OutakeConstants.wheelsConfig.getV();

        Shooter.CurrentLimits.SupplyCurrentLimitEnable = true;
        Shooter.CurrentLimits.SupplyCurrentLimit = 25;

        Shooter.MotionMagic.MotionMagicAcceleration = OutakeConstants.wheelsConfig.getAcceleration();
        Shooter.MotionMagic.MotionMagicJerk  = OutakeConstants.wheelsConfig.getJerk();

        right.getConfigurator().apply(Shooter);
        left.getConfigurator().apply(Shooter);

    }

    @Override
    public void periodic(){  
        SmartDashboard.putString("PriorityFieldZone", Field3472.getPriority());
    }

    public void setWheels(double rps)
    {
        right.setControl(motionMagic.withVelocity(rps));
        left.setControl(motionMagic.withVelocity(-rps));
    }
    public double getRPS(){
        return right.getVelocity().getValueAsDouble();
    }

    
    public void runSpeed(double speed){
        right.set(speed);
        left.set(-speed);
    }

    public void stop(){
        right.set(0);
        left.set(0);
    }

}