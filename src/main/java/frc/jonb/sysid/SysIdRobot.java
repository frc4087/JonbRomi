package frc.jonb.sysid;

import javax.swing.JOptionPane;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * A robot wrapper used for generating the SysId robot constants for a target
 * drivetrain.
 * <p>
 * To generate a SysId log, create an instance of this class with the target
 * drivetrain and follow the instructions in the dialog windows. When done, run
 * the SysId tool to analyze the generated SysId log. See
 * {@link}https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/
 * for details.
 */
public class SysIdRobot extends TimedRobot {
	/**
	 * Creates an instance.
	 * 
	 * @param drive
	 *            Shared exposed drivetrain subsystem.
	 */
	public SysIdRobot(SysIdDrivable drive) {
		ErrorMessages.requireNonNullParam(drive, "drive", "SysIdHelper");

		_drive = drive;
		_routine = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(drive::setVoltage, drive::logEntry,
						drive));
	}

	/**
	 * Returns the target drivetrain.
	 * 
	 * @return The object.
	 */
	public SysIdDrivable getDrive() {
		return _drive;
	}

	// /**
	// * Connects an XBox interface device (HID) for triggering SysId routines.
	// * Overrides any previous HID connection.
	// * <p>
	// * A routine execute only while the corresponding buttons are
	// * held down -- releasing them stops the routine immediately, for safety.
	// * Typically, routines are run in order:
	// * <l>
	// * <li>Quasistatic, forward - A + R-bumper
	// * <li>Quasistatic, backward - B + R-bumper
	// * <li>Dynamic, forward - X + R-bumper
	// * <li>Dynamic, backward - Y + R-bumper
	// * </l>
	// *
	// * @param port
	// * Driver station port index.
	// */
	// public void connectXbox(int port) {
	// CommandXboxController hid = new CommandXboxController(0);
	// hid.a().and(hid.rightBumper()).whileTrue(
	// _routine.quasistatic(SysIdRoutine.Direction.kForward));
	// hid.b().and(hid.rightBumper()).whileTrue(
	// _routine.quasistatic(SysIdRoutine.Direction.kReverse));
	// hid.x().and(hid.rightBumper())
	// .whileTrue(_routine.dynamic(SysIdRoutine.Direction.kForward));
	// hid.y().and(hid.rightBumper())
	// .whileTrue(_routine.dynamic(SysIdRoutine.Direction.kReverse));
	// _message = "Using an XBox controller:\n" +
	// " A + Rgt-Bumper - Quasistatic forward.\n" +
	// " B + Rgt-Bumper - Quasistatic backward.\n" +
	// " X + Rgt-Bumper - Dynamic forward.\n" +
	// " Y + Rgt-Bumper - Dynamic backward.\n";
	// }

	// Robot

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void robotInit() {
		new Thread(new Runnable() {
			public void run() {
				CommandXboxController hid = new CommandXboxController(0);

				showContinueOrExitDialog(""
						+ "Place the robot in an area with at least\n"
						+ "10 feet of clear space in front of it.\n\n" +
						"Then, enable the robot (e.g. set\n" +
						"Robot Status to Test mode).");

				hid.rightBumper().whileTrue(
						_routine.quasistatic(SysIdRoutine.Direction.kForward));
				showNextTestDialog("Quasistatic Forward");

				hid.rightBumper().whileTrue(
						_routine.quasistatic(SysIdRoutine.Direction.kReverse));
				showNextTestDialog("Quasistatic Reverse");

				showAllDoneDialog(""
						+ "This application is done. If all tests have been\n" +
						"run successfully to completion the SysId log is\n"
						+ "will be complete. Run the SysId tool to analyze\n"
						+ "the completed log data to determine the robot\n" +
						"characterization constants.");

				System.exit(0);
			}
		}).start();
	}

	// personal

	private final SysIdDrivable _drive;
	private final SysIdRoutine _routine;

	// class

	/**
	 * Shows a dialog with a message, a "Continue" button, and a "Exit" button.
	 * Blocks until Continue is pressed. If Exit is pressed or the dialog is
	 * closed, the application exits.
	 * 
	 * @param message
	 *            The message.
	 */
	public static void showContinueOrExitDialog(String message) {
		int result = JOptionPane.showOptionDialog(null, message + "\n\n",
				"SysIdRobot",
				JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE,
				null,
				new Object[] { "Continue", "Exit" }, "Continue");

		if (result != 0) {
			// not Continue: assume exit
			System.exit(0);
		}
	}

	/**
	 * Shows a continue-or-exit dialog prompting the user to run the next SysId
	 * test.
	 * 
	 * @param testName
	 *            The test name.
	 */
	public static void showNextTestDialog(String testName) {
		showContinueOrExitDialog(
				"Ready to run test >>> " + testName + " <<<.\n\n" +
						"Press and hold the Right Bumper button until the\n" +
						"the robot stops moving, which means the test\n" +
						"completed\n" +
						"successfully.\n\n" +
						"If there is a problem, release the button,\n" +
						"exit the application, and start over.");
	}

	/**
	 * Shows a dialog with a message, a "Done" button. he application exits when
	 * the button is pressed or the dialog is closed.
	 * 
	 * @param message
	 *            The message.
	 */
	public static void showAllDoneDialog(String message) {
		JOptionPane.showOptionDialog(null, message + "\n\n",
				"SysIdRobot",
				JOptionPane.DEFAULT_OPTION, JOptionPane.INFORMATION_MESSAGE,
				null,
				new Object[] { "Done" }, "Done");

		System.exit(0);
	}

	/**
	 * Test command that drives the robot forward at a "slow" speed. Used to
	 * test robot motor activation and HID button binding.
	 */
	public static class MyTestCommand extends Command {
		public MyTestCommand(SysIdDrivable drive) {
			_drive = drive;
			addRequirements(_drive);
		}

		@Override
		public boolean runsWhenDisabled() {
			return true;
		}

		@Override
		public void initialize() {
			System.out.println("Starting MyTestCommand");
		}

		@Override
		public void execute() {
			_drive.setVoltage(Units.Volts.of(3.0));
		}

		@Override
		public void end(boolean interrupted) {
			System.out.println("Stopping MyTestCommand");
			_drive.setVoltage(Units.Volts.of(0.0));
		}

		private SysIdDrivable _drive;
	}

}
