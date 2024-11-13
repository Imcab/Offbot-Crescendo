// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignJoystickPid;
import frc.robot.commands.AlignShoot;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.JoystickTurret;
import frc.robot.commands.Shoot;
import frc.robot.commands.feed;
import frc.robot.commands.warm;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.DriveCommands.ResetHeading;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.pitch;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.turret;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  CommandXboxController controller2 = new CommandXboxController(1);
  intake intake;
  swerve swerve;
  turret turret; 
  pitch angle;
  shooter wheels;
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    intake = new intake();
    swerve = new swerve();
    turret = new turret();
    angle = new pitch();
    wheels = new shooter();
  
    NamedCommands.registerCommand("Intake", new feed(intake,turret, 0.2, 1, false).withTimeout(1.2));
    NamedCommands.registerCommand("ShootSpeaker", new AlignShoot(angle, wheels, intake, 75.0, 48, 0.8));

    //Autos
    autoChooser.addOption("Center 3 pz", new PathPlannerAuto("cs"));
    autoChooser.addOption("Right 1 pz (don't move)", new PathPlannerAuto("rs"));
    autoChooser.addOption("Left 1 pz (don't move)", new PathPlannerAuto("ls"));

    autoChooser.addOption("Center 1 pz (leave)", new PathPlannerAuto("cout"));
    autoChooser.addOption("Left 1 pz (leave)", new PathPlannerAuto("lout"));
    autoChooser.addOption("Right 1 pz (leave)", new PathPlannerAuto("rout"));

    SmartDashboard.putData(autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    //------------------------------------------- DRIVER 1 ------------------------------------------- 
    controller.start().whileTrue(new ResetHeading(swerve));

    swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve, ()-> controller.getLeftY(),  ()-> controller.getLeftX(),  ()-> -controller.getRightX()));

    controller.a().whileTrue(new feed(intake, turret,0.2, 1, false));
    controller.b().whileTrue(new feed(intake,turret, -0.4, -1, true));

    controller.y().whileTrue(new IntakeFromShooter(wheels, intake, turret, angle, 90));
    //------------------------------------------- DRIVER 1 -------------------------------------------


    //------------------------------------------- DRIVER 2 ------------------------------------------- 
    //shooter
    controller2.a().whileTrue(new Shoot(wheels, intake, 80.0)); //Solo shooter
    controller2.povDown().toggleOnTrue(new warm(wheels, 0.3)); //warm shooter

    //turret
    turret.setDefaultCommand(new JoystickTurret(turret, ()->controller2.getRightX() * 0.7 , controller2));//Mover torreta joyStick
    controller2.rightStick().whileTrue(new AlignTurret(turret, 0.1));//Reiniciar torreta a 0

    //angulador
    controller2.leftStick().toggleOnTrue(new AlignJoystickPid(angle, ()-> -controller2.getLeftY()));

    //setpoints
    controller2.rightBumper().whileTrue(new AlignShoot(angle, wheels, intake, 75.0, 48, 0.8));
    controller2.leftBumper().whileTrue(new AlignShoot(angle, wheels, intake, 80.0, 32, 0.8));

    //Pases
    controller2.x().whileTrue(new AlignShoot(angle, wheels, intake, 60, 45 , 0.8));
    controller2.b().whileTrue(new AlignShoot(angle, wheels, intake, 72, 12 , 0.8));

    //------------------------------------------- DRIVER 2 ------------------------------------------- 


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
