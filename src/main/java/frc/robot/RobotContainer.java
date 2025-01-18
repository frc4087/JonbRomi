// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.romi.OnBoardIO;
import edu.wpi.first.wpilibj.romi.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.jonb.pathplanner.PPBridge;
import frc.jonb.pathplanner.PPDrivable;
import frc.jonb.subsystems.DiffDriveSubsystem;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.RomiDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 * <p>
 * NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the
 * hardware "overlay" that is specified when launching the wpilib-ws server on
 * the Romi raspberry pi. By default, the following are available (listed in
 * order from inside of the board to outside):
 * - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
 * - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
 * - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
 * - PWM 2 (mapped to Arduino Pin 21)
 * - PWM 3 (mapped to Arduino Pin 22)
 * Your subsystem configuration should take the overlays into account
 */
public class RobotContainer {
	/**
	 * Creates an instance.
	 */
	public RobotContainer() {
		// build subsystems
		_romiDrive = new RomiDriveSubsystem();
		_ppDrive = new DiffDriveSubsystem(_romiDrive);

		// connect PathPlanner (first)
		PPBridge.buildBridge(_ppDrive);

		// build IO
		_chooser = new SendableChooser<>();
		configChooser(_chooser);

		_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
		configButtons(_onboardIO);

		_controller = new Joystick(0);

		// default commands
		// Note: This runs unless another command is scheduled over it.
		_romiDrive.setDefaultCommand(getArcadeDriveCommand());
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
	 * then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	protected void configButtons(OnBoardIO io) {
		// Example of how to use the onboard IO
		new Trigger(io::getButtonAPressed)
				.onTrue(new PrintCommand("Button A Pressed"))
				.onFalse(new PrintCommand("Button A Released"));
	}

	/**
	 * Builds a chooser for the SmartDashboard GUI. MUST first build PPBridge.
	 */
	protected void configChooser(SendableChooser<Command> chooser) {
		chooser.setDefaultOption("Path Planner",
				new PathPlannerAuto("MyAutoPath"));
		chooser.addOption("Auto Routine Distance",
				new AutonomousDistance(_romiDrive));
		chooser.addOption("Auto Routine Time",
				new AutonomousTime(_romiDrive));

		SmartDashboard.putData(chooser);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		Command command = _chooser.getSelected();
		return command;
	}

	/**
	 * Use this to pass the teleop command to the main {@link Robot} class.
	 *
	 * @return the command to run in teleop
	 */
	public Command getArcadeDriveCommand() {
		return new ArcadeDrive(
				_romiDrive, () -> -_controller.getRawAxis(1),
				() -> -_controller.getRawAxis(2));
	}

	// personal

	private final RomiDriveSubsystem _romiDrive;
	private final PPDrivable _ppDrive;
	private final OnBoardIO _onboardIO;
	private final SendableChooser<Command> _chooser;
	private final Joystick _controller;
}
