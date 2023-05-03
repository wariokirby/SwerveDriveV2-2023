// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class WheelRotationSubsystem extends PIDSubsystem {
  private CANSparkMax angleMotor;
  private CANCoder rotationEncoder;
  private int encoderID;
  /** Creates a new WheelRotationSubsystem. */
  public WheelRotationSubsystem(int motorID , int encoderID) {//id convention motorID +10 drivr, encoder +20
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    angleMotor = new CANSparkMax(motorID , MotorType.kBrushless);
    rotationEncoder = new CANCoder(encoderID);
    rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    //TODO use configMagnetOffset to zero the module to straight
    this.encoderID = encoderID;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    angleMotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    SmartDashboard.putNumber("Wheel " + (encoderID - 20) + " angle", rotationEncoder.getAbsolutePosition());
    // Return the process variable measurement here
    return 0;
  }
}
