// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command that drives forward for a given duration at a given speed, turns
 * around, drives back to the original position, and turns around to the
 * original orientation.
 */
public class AutoDuration extends SequentialCommandGroup {
	/**
	 * Creates an instance.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param duration
	 *            Distance (s, >=).
	 */
	public AutoDuration(double speedFactor, double duration,
			DiffDriveSubsystem drivetrain) {
		addCommands(
				new DriveDuration(drivetrain, speedFactor, duration),
				new TurnDuration(drivetrain, speedFactor, duration),
				new DriveDuration(drivetrain, speedFactor, duration),
				new TurnDuration(drivetrain, speedFactor, duration));
	}
}
