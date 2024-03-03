package frc.robot;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {
  private final PS4Controller driver = new PS4Controller(0);
  public Swerve swerve = new Swerve();

  public RobotContainer() {

    TeleopSwerve Omar = new TeleopSwerve(swerve, driver);
    swerve.setDefaultCommand(Omar);

  }



   public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("Example Auto");
  }

}