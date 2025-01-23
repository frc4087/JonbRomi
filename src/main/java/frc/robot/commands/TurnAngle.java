// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.math.ContinuousRotation2d;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that turns CCW for a given angle at a given speed.
 */
public class TurnAngle extends Command {
	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1]. Sign is ignored --
	 *            it will be forced to match that of the angle.
	 * @param angle
	 *            Angle (+/-deg).
	 */
	public TurnAngle(DiffDriveSubsystem drive, double speedFactor,
			double angle) {
		_drive = drive;
		_speed = Math.signum(angle) * Math.abs(speedFactor);
		_angle = angle;
		addRequirements(drive);

		_continuous = new ContinuousRotation2d(
				() -> _drive.getPose().getRotation());
	}

	@Override
	public void initialize() {
		_drive.arcadeDrive(0, 0);
		_drive.resetPose(Pose2d.kZero);
	}

	@Override
	public void execute() {
		_drive.arcadeDrive(0, _speed);
	}

	@Override
	public void end(boolean interrupted) {
		// stop drive
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		double angle = _continuous.get().getDegrees();
		double error = Math.signum(_angle) * (_angle - angle);
		// System.out.println("TurnAngle: " +
		// 		"goal=" + _angle + " now=" + angle + " err=" + error);
		return error <= 0.0;
	}

	// personal

	private final DiffDriveSubsystem _drive;
	private final double _angle;
	private final double _speed;

	private ContinuousRotation2d _continuous;
}
