// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that drives forward for a given duration at a given speed.
 */
public class DriveDuration extends Command {
	private final double m_duration;
	private final double m_speed;
	private final DiffDriveSubsystem m_drive;
	private long m_startTime;

	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param duration
	 *            Distance (s, >=0).
	 */
	public DriveDuration(DiffDriveSubsystem drive, double speedFactor, double duration) {
		m_speed = speedFactor;
		m_duration = duration * 1000;
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_startTime = System.currentTimeMillis();
		m_drive.arcadeDrive(0, 0);
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
		return (System.currentTimeMillis() - m_startTime) >= m_duration;
	}
}
