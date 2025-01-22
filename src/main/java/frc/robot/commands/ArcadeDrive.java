// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.jonb.subsystems.DiffDriveSubsystem;

/**
 * Command supporting "arcade" style (forward and turn) drive control, such
 * as from a hand controller. Runs until explicitely terminated.
 */
public class ArcadeDrive extends Command {
	private final DiffDriveSubsystem m_drivetrain;
	private final Supplier<Double> m_forwardSource;
	private final Supplier<Double> m_turnSource;

	/**
	 * Creates an instance.
	 * 
	 * @param drivetrain
	 *            The target drivetrain.
	 * @param forwardSource
	 *            Supplier of forward speed factor relative to max [-1, +1].
	 * @param turnSource
	 *            Supplier of CCW turn speed factor relative to max [-1, +1].
	 */
	public ArcadeDrive(
			DiffDriveSubsystem drivetrain,
			Supplier<Double> forwardSource,
			Supplier<Double> turnSource) {
		m_drivetrain = drivetrain;
		m_forwardSource = forwardSource;
		m_turnSource = turnSource;
		addRequirements(drivetrain);
	}

	// Command

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		m_drivetrain.arcadeDrive(m_forwardSource.get(),
				m_turnSource.get());
	}

	@Override
	public void end(boolean interrupted) {
		// nothing to do
	}

	@Override
	public boolean isFinished() {
		return false; // run forever
	}
}
