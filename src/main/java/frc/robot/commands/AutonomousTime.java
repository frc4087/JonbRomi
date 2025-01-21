// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RomiDriveSubsystem;

public class AutonomousTime extends SequentialCommandGroup {
  	/**
	 * Command that drives forward for a given time at a given speed, turns
	 * around, drives back to the original position, and turns around to the
	 * original orientation.
	 *
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param speedFactor
	 *            Speed factor relative to max [-1, +1].
	 * @param distance
	 *           Distance (m, >0).
	 */
	public AutonomousTime(double speedFactor, double seconds,
			RomiDriveSubsystem drivetrain) {
    addCommands(
        new DriveTime(speedFactor, seconds, drivetrain),
        new TurnTime(speedFactor, seconds, drivetrain),
        new DriveTime(speedFactor, seconds, drivetrain),
        new TurnTime(speedFactor, seconds, drivetrain));
  }
}
