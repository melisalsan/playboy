package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {

    private Swerve swerve;
    private double yukari_asagi;
    private double sag_sol;
    private double donme;
    private boolean zeroGyro;
    private PS4Controller driver;
    private PIDController rotationController;

    public TeleopSwerve(Swerve swerve, PS4Controller driver) {
        addRequirements(swerve);

        rotationController = new PIDController(0.01, 0, 0 );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(3);

        this.swerve = swerve;
        this.driver = driver;
        
    }

    @Override
    public void execute() {

        this.yukari_asagi =driver.getLeftY();
        this.sag_sol = driver.getLeftX();
        this.donme = driver.getRightX();
        this.zeroGyro = driver.getL1Button();
      
      if(zeroGyro) {

        swerve.zeroGyro();
      }

      double translationVal  = MathUtil.applyDeadband(yukari_asagi, Constants.Swerve.axisDeadBand);
      double strafeVal  = MathUtil.applyDeadband(sag_sol, Constants.Swerve.axisDeadBand);
      double  rotationVal  = MathUtil.applyDeadband(donme, Constants.Swerve.axisDeadBand);

      swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal, Constants.Swerve.fieldOriented,
        true
      );
      System.out.println(yukari_asagi + " " + sag_sol + " " + donme);
      
    
    
    }

    
}


