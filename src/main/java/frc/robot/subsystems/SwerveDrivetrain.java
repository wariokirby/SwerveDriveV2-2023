// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class SwerveDrivetrain extends SubsystemBase {
  //locations assuming 32 x 28 robot or .831 x .711 MUST BE MEASURED
  //max theoretical speed 12 ft/sec or 3.658 m/s
  //max rotational speed: circumference 100.53in or 2.553m, .69s for 2pi radians or 9.1 radians/sec
  private final double MAX_ROTATIONAL_SPEED = 9.1;// radians/s
  private SwerveDriveKinematics kinematics;
 
  private WheelDriveSubsystem[] swerveDriveModules;
  private WheelRotationSubsystem[] swerveRotationModules;
  private ChassisSpeeds speeds;
  private SwerveModuleState[] moduleStates;
  private SwerveModuleState[] optimizedStates;

  
  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(int[] moduleIDs) {//order same as kinematics
    var frontLeftLocation = new Translation2d(.4 , .35);
    var frontRightLocation = new Translation2d(.4 , -.35);
    var backLeftLocation = new Translation2d(-.4 , .35);
    var backRightLocation = new Translation2d(-.4 , -.35);
    kinematics = new SwerveDriveKinematics(frontLeftLocation , frontRightLocation , backLeftLocation , backRightLocation);
    for(int index = 0; index < 12; index+=3){
      swerveDriveModules[index] = new WheelDriveSubsystem(moduleIDs[index]);
      swerveRotationModules[index] = new WheelRotationSubsystem(moduleIDs[index + 1], moduleIDs[index + 2]);
    }
    moduleStates = new SwerveModuleState[4];
    optimizedStates = new SwerveModuleState[moduleStates.length];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double move , double strafe , double turn){
    move *= MAX_SPEED;
    strafe *= -MAX_SPEED;
    turn *= -MAX_ROTATIONAL_SPEED;
    speeds = new ChassisSpeeds(move , strafe , turn);
    moduleStates = kinematics.toSwerveModuleStates(speeds);
    for(int index = 0; index < moduleStates.length; index++){
      optimizedStates[index] = SwerveModuleState.optimize(moduleStates[index], new Rotation2d(swerveRotationModules[index].getMeasurement()));
      swerveDriveModules[index].setSetpoint(optimizedStates[index].speedMetersPerSecond);
      swerveRotationModules[index].setSetpoint(optimizedStates[index].angle.getDegrees());
    }//end for
  
  }//end drive

}
