// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.subsystems.DiffDrivable;

/**
 * A DriveSubsystem for a Romi robot.
 */
public class RomiDriveSubsystem extends SubsystemBase implements DiffDrivable {
  public static final double WHEEL_TRACKWIDTH_M = 0.141;
  public static final double WHEEL_ENCODER_CPR = 1440.0;
  public static final double WHEEL_DIAMETER_M = 0.07;
  public static final double WHEEL_CIRCUMFERENCE_M = Math.PI * WHEEL_DIAMETER_M;
  public static final double WHEEL_RPS_MAX = 150.0 / 60.0; // from WPI docs
  public static final double WHEEL_MPS_MAX = WHEEL_RPS_MAX
      * WHEEL_CIRCUMFERENCE_M;

  /** Creates a new Drivetrain. */
  public RomiDriveSubsystem() {
    _subsystems.add(this);

    _rightMotor.setInverted(true);

    _leftEncoder
        .setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
    _rightEncoder
        .setDistancePerPulse(WHEEL_CIRCUMFERENCE_M / WHEEL_ENCODER_CPR);
    resetEncoders();
  }

  @Override
  public double getTrackWidth() {
    return WHEEL_TRACKWIDTH_M;
  }

  @Override
  public double getSpeedMax() {
    return WHEEL_MPS_MAX;
  }

  /**
   * Sets robot chassis relative speed factors.
   * 
   * @param forwardFactor
   *        Forward speed factor [-1,+1].
   * @param rotateSpeed
   *        Rotation CCW speed factor [-1,+1].
   */
  @Override
  public void arcadeDrive(double forwardFactor, double ccwFactor) {
    _diffDrive.arcadeDrive(forwardFactor, ccwFactor);
  }

  /**
   * Sets robot wheel speed factors.
   * 
   * @param leftFactor
   *        Left wheel forward speed factor [-1,+1].
   * @param rightFactor
   *        Right wheel forward speed factor [-1,+1].
   */
  @Override
  public void tankDrive(double leftFactor, double rightFactor) {
    _diffDrive.tankDrive(leftFactor, rightFactor, false);
  }

  @Override
  public void resetEncoders() {
    _leftEncoder.reset();
    _rightEncoder.reset();
  }

  /**
   * Gets the current distance traveled by the left wheel, as indicated by its
   * wheel encoders.
   * 
   * @return Value (m).
   */
  @Override
  public double getLeftDistance() {
    return _leftEncoder.getDistance();
  }

  /**
   * Gets the current distance traveled by the right wheel, as indicated by its
   * wheel encoders.
   * 
   * @return Value (m).
   */
  @Override
  public double getRightDistance() {
    return _rightEncoder.getDistance();
  }

  /**
   * Gets the current dvelocity the left wheel, as indicated by its
   * wheel encoders.
   * 
   * @return Value (m/s).
   */
  @Override
  public double getLeftVelocity() {
    return _leftEncoder.getRate();
  }

  /**
   * Gets the current velocity of the right wheel, as indicated by its
   * wheel encoders.
   * 
   * @return Value (m/s).
   */
  @Override
  public double getRightVelocity() {
    return _rightEncoder.getRate();
  }

  /**
   * Resets the gyro, with zero angles corresponding to the current orientation.
   */
  @Override
  public void resetGyro() {
    _gyro.reset();
  }

  /**
   * Gets the current CCW rotation of the chassis about the vertical/Z axis
   * relative to resetGyro().
   * 
   * @return The value.
   */
  @Override
  public Rotation2d getRotationZ() {
    return Rotation2d.fromDegrees(-_gyro.getAngleZ());
  }

  /**
   * Gets the subsystems required to support this subsystem, including this one.
   * 
   * @return Temp output group.
   */
  @Override
  public List<Subsystem> getSubsystems() {
    return Collections.unmodifiableList(_subsystems);
  }
  // personal

  private List<Subsystem> _subsystems = new ArrayList<>();

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark _leftMotor = new Spark(0);
  private final Spark _rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder _leftEncoder = new Encoder(4, 5);
  private final Encoder _rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive _diffDrive = new DifferentialDrive(
      _leftMotor::set, _rightMotor::set);

  // Set up the RomiGyro
  private final RomiGyro _gyro = new RomiGyro();
}
