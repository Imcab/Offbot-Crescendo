package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Gains;

public class Constants {
  
  public class FieldConstants {

    public static final double fieldLength = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);

    public static final Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

      public static final class Amp {
        public static final Translation2d ampTapeTopCorner =
            new Translation2d(Units.inchesToMeters(130.0), Units.inchesToMeters(305.256));
        public static final double ampBottomY = fieldWidth - Units.inchesToMeters(17.75);
      }

    /** Staging locations for each note */
    public static final class StagingLocations {
        public static final double centerlineX = fieldLength / 2.0;

      // need to update
      public static final double centerlineFirstY = Units.inchesToMeters(29.638);
      public static final double centerlineSeparationY = Units.inchesToMeters(66);
      public static final double spikeX = Units.inchesToMeters(114);
      // need
      public static final double spikeFirstY = Units.inchesToMeters(161.638);
      public static final double spikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] centerlineTranslations = new Translation2d[5];
      public static final Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] =
              new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }
  }

  public class DriveConstants {

    public static final double kP = 6.8; 

    public static final double kPSIM = 10.0;

    public static final double WHEELRADIUS = Units.inchesToMeters(2.0);

    public static final double WHEELDIAMETER = Units.inchesToMeters(4.0);

    public static final double maxspeedMetersPerSecond = 5.7912;

    public static final double DriveReduction = 5.36;
    public static final double TurnReduction = 18.75;

    public static final class frontLeft{

        public static final int DrivePort = 6; 
        public static final int TurnPort = 5; 
        public static final int EncPort = 3;
        public static final double offset = 46;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;


    }

    public static final class frontRight{

        public static final int DrivePort = 8; 
        public static final int TurnPort = 7; 
        public static final int EncPort = 2; 
        public static final double offset = 69.6; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 2; 
        public static final int TurnPort = 1; 
        public static final int EncPort = 0; 
        public static final double offset = 0;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 1; 
        public static final double offset = 60.2; 
 
        public static final boolean DrivemotorReversed = false;
        public static final boolean TurnmotorReversed = true;


    }
}

public class ShooterConstants {

    public static final class TurretConstants{

        public static final double TurretReduction = 27.5; 
        //0.001959
        public static final Gains turretConfig = new Gains(0.001959, 0.0, 0.000);
        public static final Gains limeConfig = new Gains(0.05, 0.0, 0.00);

        public static final Gains simTurretConfig = new Gains(4, 0.0, 0.08);

        public static final int TurretPort = 19;
        public static final int EncDIOPORT = 4;
        public static final double offset = 0;
        public static final boolean TurretReversed = false;
        
    }
    public static final class AngleShooterConstants{

        public static final double SpeakerDefault = 19;

        public static final double AngleGearing = 0.4848; 

        public static final Gains angleConfig = new Gains(0.025, 0.01, 0.0015);

        public static final Gains limelightConfig = new Gains(0.00515, 0.001, 0.00000);

        public static final Gains simAngleConfig = new Gains(4, 0, 2.0);

        public static final int AngleShooterPort = 12;
        public static final int EncPort = 3;
        public static final double offset = 58.3;
        public static final boolean AngleMotorReversed = false;

    }
     public static final class OutakeConstants{

        public static final double WheelReduction = 1; //1:1

        public static final Gains wheelsConfig = new Gains(0.6, 0.0, 0.01, 0.4, 0.12, 400, 4000);

        public static final int LeftWHeelsPort = 13;
        public static final int RightWHeelsPort = 14;
        public static final boolean LeftMotorReversed = true; 
        public static final boolean RightMotorReversed = false;

        ////////////indexador////////////////
        public static final int IndexerPort = 15;
        public static final boolean IndexerMotorReversed = true;
        public static final int BEAMSENSOR_RECIEVER_DIOPIN = 4;
        
    }
}

public static final class IntakeConstants{

    public static final double kp = 0.11; 
    public static final double kd = 0.002; 
    public static final double kv = 0.243;
    public static final double ks = 0.19; 
    public static final int Intakeport = 9; 
    public static final boolean IntakeMotorReversed = false;  
    public static final double ratio = 12;
    
}

}
