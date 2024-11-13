package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class modulespark {
    CANSparkMax driveSparkMax, turnSparkMax;
    AnalogInput AbsoluteEncoder;
    RelativeEncoder enc_drive, enc_turn;

    boolean isDriveMotorInverted;
    boolean isTurnMotorInverted;

    private final PIDController turnPID;
    private final PIDController drivePID;
    private final SimpleMotorFeedforward drivFeedforward;
    private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
    private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

    double Offset;

    public modulespark(int index){

        drivePID = new PIDController(0.05, 0.0, 0.0);
        drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        turnPID = new PIDController(DriveConstants.kP, 0.0, 0.0);

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(DriveConstants.frontLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.frontLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.frontLeft.EncPort);
            isDriveMotorInverted = DriveConstants.frontLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontLeft.TurnmotorReversed;
            Offset = DriveConstants.frontLeft.offset;
            

            break;
          case 1:
            driveSparkMax = new CANSparkMax(DriveConstants.frontRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.frontRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.frontRight.EncPort);
            isDriveMotorInverted = DriveConstants.frontRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.frontRight.TurnmotorReversed;
            Offset = DriveConstants.frontRight.offset; 
            

            break;
          case 2:
            driveSparkMax = new CANSparkMax(DriveConstants.backLeft.DrivePort, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.backLeft.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.backLeft.EncPort);
            isDriveMotorInverted = DriveConstants.backLeft.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backLeft.TurnmotorReversed;
            Offset = DriveConstants.backLeft.offset;
            

            break;
          case 3:
            driveSparkMax = new CANSparkMax(DriveConstants.backRight.DrivePort, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(DriveConstants.backRight.TurnPort, MotorType.kBrushless);
            AbsoluteEncoder = new AnalogInput(DriveConstants.backRight.EncPort);
            isDriveMotorInverted = DriveConstants.backRight.DrivemotorReversed;
            isTurnMotorInverted = DriveConstants.backRight.TurnmotorReversed;
            Offset = DriveConstants.backRight.offset;
            

            break;
          default:
            throw new RuntimeException("Invalid module index");
        }

        driveSparkMax.restoreFactoryDefaults();
        turnSparkMax.restoreFactoryDefaults();

        driveSparkMax.setCANTimeout(250);
        turnSparkMax.setCANTimeout(250);

        enc_drive = driveSparkMax.getEncoder();
        enc_turn = driveSparkMax.getEncoder();

        driveSparkMax.setInverted(isDriveMotorInverted);
        turnSparkMax.setInverted(isTurnMotorInverted);

        driveSparkMax.setSmartCurrentLimit(30);
        turnSparkMax.setSmartCurrentLimit(30);
        driveSparkMax.enableVoltageCompensation(12.0);
        turnSparkMax.enableVoltageCompensation(12.0);

        enc_drive.setPosition(0.0);
        enc_drive.setMeasurementPeriod(10);
        enc_drive.setAverageDepth(2);

        enc_turn.setPosition(0.0);
        enc_turn.setMeasurementPeriod(10);
        enc_turn.setAverageDepth(2);
        
        driveSparkMax.setCANTimeout(0);
        turnSparkMax.setCANTimeout(0);

        driveSparkMax.burnFlash();
        turnSparkMax.burnFlash();
    }

    public void periodic(){
        if (angleSetpoint != null) {
            turnSparkMax.setVoltage(
                turnPID.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
      
            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
              // Scale velocity based on turn error
              //
              // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
              // towards the setpoint, its velocity should increase. This is achieved by
              // taking the component of the velocity in the direction of the setpoint.
              double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnPID.getPositionError());
      
              // Run drive controller
              double velocityRadPerSec = adjustSpeedSetpoint / DriveConstants.WHEELRADIUS;
              driveSparkMax.setVoltage(
                  drivFeedforward.calculate(velocityRadPerSec)
                      + drivePID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / DriveConstants.DriveReduction, velocityRadPerSec));
            }
          }
          SmartDashboard.putNumber("Enc" , AngleEncoder().getDegrees());
    }

    public Rotation2d AngleEncoder(){

      double encoderBits = AbsoluteEncoder.getValue();
      double angleEncoder = (encoderBits * 360) / 4096;

      return Rotation2d.fromDegrees(angleEncoder - Offset);

    }
    public Rotation2d getAngle(){
        return new Rotation2d(AngleEncoder().getRadians());
    }

    public Rotation2d AngleEncoderODOMETRY(){
        double angleFixed = getAngle().getDegrees();
        return Rotation2d.fromDegrees(angleFixed);
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveSparkMax.setVoltage(desiredState.speedMetersPerSecond);
  }

  public double getDrivePositionMeters(){
    return Units.rotationsToRadians(enc_drive.getPosition()) / DriveConstants.DriveReduction * DriveConstants.WHEELRADIUS;
  }  
  public double getDriveVelocityMetersxSec(){
    return Units.rotationsPerMinuteToRadiansPerSecond(enc_drive.getVelocity()) / DriveConstants.DriveReduction;
  }
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePositionMeters(), new Rotation2d(AngleEncoderODOMETRY().getRadians()));
  }
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocityMetersxSec(), new Rotation2d(AngleEncoderODOMETRY().getRadians()));
  }

  public void stop() {
    driveSparkMax.set(0.0);
    turnSparkMax.set(0.0);
     // Disable closed loop control for turn and drive
     angleSetpoint = null;
     speedSetpoint = null;
  }





}