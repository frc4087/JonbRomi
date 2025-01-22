// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that turns CCW for a given angle at a given speed.
 */
public class TurnAngle extends Command {
	private final DiffDriveSubsystem m_drive;
	private final double m_angle;
	private final double m_speed;

	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param angle
	 *            Angle (deg, >0).
	 */
	public TurnAngle(DiffDriveSubsystem drive, double speedFactor, double angle) {
		m_angle = angle;
		m_speed = speedFactor;
		m_drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		m_drive.arcadeDrive(0, 0);
		m_drive.resetPose(Pose2d.kZero);
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
		double angle = m_drive.getPose().getRotation().getDegrees();
		return Math.signum(m_angle) * Math.abs(m_angle - angle) <= 0.0;
	}
}
