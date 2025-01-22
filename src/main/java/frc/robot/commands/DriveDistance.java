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
	private final DiffDriveSubsystem m_drive;
	private final double m_distance;
	private final double m_speed;

	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param meters
	 *            Distance (m, >=0).
	 */
	public DriveDistance(DiffDriveSubsystem drive, double speedFactor,
			double meters) {
		m_distance = meters;
		m_speed = speedFactor;
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_drive.arcadeDrive(0.0, 0.0);
		m_drive.resetPose(Pose2d.kZero);
	}

	@Override
	public void execute() {
		m_drive.arcadeDrive(m_speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		// stop drive
		m_drive.arcadeDrive(0, 0);
	}

	@Override
	public boolean isFinished() {
		double distance = m_drive.getPose().getX();
		return Math.signum(m_distance) * Math.abs(m_distance - distance) <= 0.0;
	}
}
