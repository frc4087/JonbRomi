package frc.jonb.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DiffDrivable extends Subsystem {

	/**
	 * Gets the trackwidth between the wheels. Intended for kinematics.
	 * @return The width (m).
	 */
	double getTrackWidth();

	/**
	 * Gets the maximum wheel/chassis speed. Intended for computing 
	 * speed factors.
	 * @return The speed (m/s).
	 */
	double getSpeedMax();

    /**
     * Sets robot chassis speed factors.
     * 
     * @param forwardFactor
     *        Forward speed factor [-1,+1].
     * @param rotateSpeed
     *        Rotation CCW speed factor [-1,+1].
     */
    void arcadeDrive(double forwardFactor, double ccwFactor);

    /**
     * Sets robot wheel speed factors.
     * 
     * @param leftFactor
     *        Left wheel forward speed factor [-1,+1].
     * @param rightFactor
     *        Right wheel forward speed factor [-1,+1].
     */
    void tankDrive(double leftFactor, double rightFactor);

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
     * Gets the current distance traveled by the right wheel, as indicated by its
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
     * Resets the gyro, with zero angles corresponding to the current orientation.
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
     * Gets the subsystems required to support this subsystem, including this one.
     * 
     * @return Temp output group.
     */
    List<Subsystem> getSubsystems();
    // personal

}