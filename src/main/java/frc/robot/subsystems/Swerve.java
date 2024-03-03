// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  
  private final Pigeon2 gyro1;


  private SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;

  private Field2d field;
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  public Swerve() {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveWithChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    gyro1 = new Pigeon2(Constants.Swerve.pigeon1, "Civ Civ");

    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), pos);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, true, Constants.Swerve.Mod0.constants),
          new SwerveModule(1,false, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, false, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, false, Constants.Swerve.Mod3.constants)
        };

    field = new Field2d();
  }

  public static SwerveModulePosition[] pos = {
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0))
  };

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Calculate the chassis speeds based on whether the input is field-relative.
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getYaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    
    // Update the lastChassisSpeeds for later retrieval or use.
    lastChassisSpeeds = chassisSpeeds;

    // Convert chassis speeds to individual swerve module states.
    SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Normalize the wheel speeds to not exceed the maximum speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // Command each swerve module to move to the desired state.
    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }

    
  }
  public ChassisSpeeds getChassisSpeeds(){
    return lastChassisSpeeds;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  public void zeroGyro() {
    gyro1.setYaw(0);

  }

  public Rotation2d getYaw() {
    StatusSignal<Double> gyroYaw = gyro1.getYaw();
    Rotation2d yawRotation;
    if (Constants.Swerve.invertGyro) {
        yawRotation = Rotation2d.fromDegrees(360 - gyroYaw.getValue());
    } else {
        yawRotation = Rotation2d.fromDegrees(gyroYaw.getValue());
    }
    return yawRotation;
}

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("gyro ", getYaw().getDegrees());
    SmartDashboard.putBoolean("isOriented ", Constants.Swerve.fieldOriented);
    SmartDashboard.putBoolean("isSlow ", Constants.Swerve.slow);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAngle().getDegrees());

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    // Convert chassis speeds to swerve module states
    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

    // Desaturate wheel speeds if necessary
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);

    // Set the state for each module
    for (int i = 0; i < mSwerveMods.length; i++) {
        mSwerveMods[i].setDesiredState(states[i], false); // Assuming false for open loop control, adjust as needed
    }


    }

}
