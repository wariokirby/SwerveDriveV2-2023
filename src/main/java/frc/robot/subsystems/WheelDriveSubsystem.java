// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import static frc.robot.Constants.*;


public class WheelDriveSubsystem extends PIDSubsystem {
  private CANSparkMax driveMotor;
  private RelativeEncoder driveEncoder;
  private SimpleMotorFeedforward driveFF;
  private int motorID;

  private final double GEAR_RATIO = .12285;//assumes mk4 standard module with an 8.14:1 gear ratio
  private final double WHEEL_CIRCUMFERENCE = .1016 * Math.PI;//4 inch wheels in meters
  private final double DRIVE_FEEDFORWARD_KS = 0.05;//TODO probably need to find a way to measure this 
  private final double DRIVE_FEEDFORWARD_KV = 12.0 / MAX_SPEED;
  /** Creates a new WheelDriveSubsystem. */
  public WheelDriveSubsystem(int motorID) {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
        this.motorID = motorID;
        driveMotor = new CANSparkMax(motorID , MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(GEAR_RATIO);
        driveEncoder.setVelocityConversionFactor(GEAR_RATIO);

        driveFF = new SimpleMotorFeedforward(DRIVE_FEEDFORWARD_KS, DRIVE_FEEDFORWARD_KV);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    driveMotor.setVoltage(output + driveFF.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("Wheel Velocity " + motorID, getRate());
    SmartDashboard.putNumber("Wheel Distance " + motorID, getDistance());
    // Return the process variable measurement here
    return getRate();
  }

  public double getRate(){
    return driveEncoder.getVelocity() * WHEEL_CIRCUMFERENCE;
  }

  public double getDistance(){
    return driveEncoder.getPosition() * WHEEL_CIRCUMFERENCE;
  }

  public void resetDistance(){
    driveEncoder.setPosition(0);
  }
}
