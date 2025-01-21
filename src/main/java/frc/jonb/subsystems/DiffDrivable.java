package frc.jonb.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for a basic differential drivetrain.
 */
public interface DiffDrivable extends Subsystem {

	/**
	 * Gets the trackwidth between the wheels. Intended for kinematics.
	 * 
	 * @return The width (m).
	 */
	double getTrackWidth();

	/**
	 * Gets the maximum linear velocity of the drive wheels. Intended for
	 * normalizing speeds and computing speed factors.
	 *
	 * @return Velocity (m/s).
	 */
	double getWheelVelocityMax();

	/**
	 * Sets the desired linear velocity of the drive wheels.)
	 * 
	 * @param leftMps
	 *            Left wheel velocity (m/s).
	 * @param rightMps
	 *            Right wheel velocity (m/s).
	 */
	void setWheelVelocity(double leftMps, double rightMps);

	/**
	 * Resets the wheel encoders to zero distance.
	 */
	void resetEncoders();

	/**
	 * Gets the current distance traveled by the left wheel, as indicated by its
	 * wheel encoders.
	 * 
	 * @return Value (m).
	 */
	double getLeftDistance();

	/**
	 * Gets the current distance traveled by the right wheel, as indicated by
	 * its
	 * wheel encoders.
	 * 
	 * @return Value (m).
	 */
	double getRightDistance();

	/**
	 * Gets the current velocity of the left wheel, as indicated by its
	 * wheel encoders.
	 * 
	 * @return Value (m/s).
	 */
	double getLeftVelocity();

	/**
	 * Gets the current velocity of the right wheel, as indicated by its
	 * wheel encoders.
	 * 
	 * @return Value (m/s).
	 */
	double getRightVelocity();

	/**
	 * Resets the gyro, with zero angles corresponding to the current
	 * orientation.
	 */
	void resetGyro();

	/**
	 * Gets the current CCW rotation of the chassis about the vertical/Z axis
	 * relative to resetGyro().
	 * 
	 * @return The value.
	 */
	Rotation2d getRotationZ();

	/**
	 * Gets the subsystems required to support this subsystem, including this
	 * one.
	 * 
	 * @return Temp output group.
	 */
	Subsystem[] getSubsystems();
}