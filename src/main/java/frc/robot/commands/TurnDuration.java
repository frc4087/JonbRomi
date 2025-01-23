// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/*
 * Creates a new TurnTime command. This command will turn your robot for a
 * desired rotational speed and time.
 */
public class TurnDuration extends Command {
	private final double m_duration;
	private final double m_speed;
	private final DiffDriveSubsystem m_drive;
	private long m_startTime;

	/**
	 * Command that turns CCW for a given duration at a given speed.
	 */
	public TurnDuration(DiffDriveSubsystem drive, double speedFactor,
			double duration) {
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
		m_drive.arcadeDrive(0, m_speed);
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
