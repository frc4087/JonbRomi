// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that drives forward a given distance at a given speed.
 */
public class DriveDistance extends Command {
	private final DiffDriveSubsystem _drive;
	private final double _distance;
	private final double _speed;

	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1]. Sign is ignored --
	 *            it will be forced to match that of the distance.
	 * @param distance
	 *            Distance (+/-m).
	 */
	public DriveDistance(DiffDriveSubsystem drive, double speedFactor,
			double distance) {
		_speed = Math.signum(distance) * Math.abs(speedFactor);
		_distance = distance;
		_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		_drive.arcadeDrive(0.0, 0.0);
		_drive.resetPose(Pose2d.kZero);
	}

	@Override
	public void execute() {
		_drive.arcadeDrive(_speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		// stop drive
		_drive.arcadeDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		double distance = _drive.getPose().getX();
		double error = Math.signum(_distance) * (_distance - distance);
		// System.out.println("DriveDistance: " +
		// 		"goal=" + _distance + " now=" + distance + " err=" + error);
		return error <= 0;
	}
}
