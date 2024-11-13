package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants.TurretConstants;

public class turret extends SubsystemBase{
    private final CANSparkMax Turret;
    private final RelativeEncoder enc_turret;
    private final SparkAbsoluteEncoder ThroughBore;
    private final boolean TurretReversed;

    private final PIDController pid;

    public static final double TURRET_LIMIT = 1.2;

    public turret(){

        pid = new PIDController(TurretConstants.turretConfig.getP(), TurretConstants.turretConfig.getI(), TurretConstants.turretConfig.getD());

        Turret = new CANSparkMax(TurretConstants.TurretPort, MotorType.kBrushless);
        TurretReversed = TurretConstants.TurretReversed;
        ThroughBore = Turret.getAbsoluteEncoder();
 
        Turret.restoreFactoryDefaults();

        Turret.setCANTimeout(250);

        enc_turret = Turret.getEncoder();

        Turret.setInverted(TurretReversed);

        Turret.enableVoltageCompensation(12.0);

        Turret.setCANTimeout(0);
        
        Turret.burnFlash();

        
    }

    public void periodic(){

        SmartDashboard.putNumber("Laps", getLaps());
        SmartDashboard.putNumber("Encoder Torreta", getDegrees());
        SmartDashboard.putBoolean("TurretLimit" , Turretlimit());
    }

  
    public double getDegrees(){
        return Units.rotationsToDegrees(ThroughBore.getPosition());
    }

     public void setTurret(double value){
        Turret.set(value);
     }
     public double getLaps(){
        return enc_turret.getPosition()/TurretConstants.TurretReduction;
     }
     
     public void runTurret(double angle){ 
        setTurret(pid.calculate(getLaps() * 360, angle));
  
     }
     
     public void runSpeed(double speed){
        setTurret(speed);
     }
     
     public void stop() {
        setTurret(0.0);
        
    }
    public boolean Turretlimit(){
     double MAX_LAPS_FORWARD = TURRET_LIMIT;   // Forward limit
     double MAX_LAPS_BACKWARD = -TURRET_LIMIT; // Backward limit
     return (getLaps() >= MAX_LAPS_FORWARD || getLaps() <= MAX_LAPS_BACKWARD);
    } 
}