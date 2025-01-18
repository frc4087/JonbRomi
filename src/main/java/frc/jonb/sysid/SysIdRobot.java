package frc.jonb.sysid;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
				showContinueOrExitDialog(""
						+ "Place the robot in an area with at least\n"
						+ "10 feet of clear space in front of it.");

				showContinueOrExitDialog(""
						+ "Enable the robot (e.g. set\n" +
						"Robot Status to Test mode).");

				int runKey = KeyEvent.VK_X;
				runTest("1 of 4",
						"Quasistatic Forward", runKey,
						_routine.quasistatic(SysIdRoutine.Direction.kForward));
				runTest("2 of 4",
						"Quasistatic Reverse", runKey,
						_routine.quasistatic(SysIdRoutine.Direction.kReverse));
				runTest("3 of 4",
						"Dynamic Forward", runKey,
						_routine.dynamic(SysIdRoutine.Direction.kForward));
				runTest("4 of 4",
						"Dynamic Forward", runKey,
						_routine.dynamic(SysIdRoutine.Direction.kReverse));

				JOptionPane.showMessageDialog(null,
						"All SysId tests are done and the SysId log\n" +
								"should be complete. Run the SysId tool to analyze\n"
								+ "the completed log data to determine the robot\n"
								+
								"characterization constants.",
						"SysIdRobot", JOptionPane.PLAIN_MESSAGE);

				System.exit(0);
			}
		}).start();
	}

	// personal

	/**
	 * Runs a SysId test by notifying the user to press and hold a given
	 * keyboard key.
	 * <p>
	 * Not a static method because mutable state needed for enclosed anonymous
	 * class.
	 * 
	 * @param sequenceId
	 *            Indicates the test number in the sequence. Should be of the
	 *            form "2 of 6".
	 * @param testName
	 *            Name of the test.
	 * @param runKey
	 *            Key code (KeyEvent.VK_???) used to run the test.
	 * @param test
	 *            The test command created using SysIdRoutine.
	 */
	public void runTest(String sequenceId, String testName, int runKey,
			Command test) {

		_isTestStarted = false;

		// build dialog
		JPanel panel = new JPanel();
		panel.add(new JLabel(
				"<html>TEST (" + sequenceId + "): " + testName + "<br><br>" +
						"Press and hold keyboard key [ "
						+ KeyEvent.getKeyText(runKey) + " ] until the<br>" +
						"the robot stops moving (test is done).<br><br>" +
						"If there is a problem release the key<br>" +
						"to stop the test immediately.<br><br>"));

		// listen for test run key
		panel.addKeyListener(new KeyListener() {
			@Override
			public void keyTyped(KeyEvent evt) {
				// do nothing
			}

			@Override
			public void keyPressed(KeyEvent evt) {
				if (evt.getKeyCode() == runKey) {
					if (_isTestStarted) {// handle auto-repeat
						return; // do nothing
					}
					_isTestStarted = true;

					System.out.println("Starting test [ " + testName + " ].");
					CommandScheduler.getInstance().schedule(test);
				}
			}

			@Override
			public void keyReleased(KeyEvent evt) {
				// Handle key released events
				if (evt.getKeyCode() == runKey) {
					System.out.println("Stopping test [ " + testName + " ].");
					CommandScheduler.getInstance().cancel(test);
					assureTestFinished(testName, test);
				}
			}
		});

		panel.setFocusable(true);
		panel.requestFocusInWindow();

		// wait until user is done
		int result = JOptionPane.showOptionDialog(null,
				panel,
				"SysIdRobot",
				JOptionPane.DEFAULT_OPTION,
				JOptionPane.INFORMATION_MESSAGE,
				null,
				new Object[] { "Next Test", "Exit" }, null);

		if (result != 0) {
			// user chose to not Continue
			System.exit(0);
		}

		// if log is incomplete, quit
		assureTestStarted(testName, test, _isTestStarted);
		assureTestFinished(testName, test);
	}

	private final SysIdDrivable _drive;
	private final SysIdRoutine _routine;

	private boolean _isTestStarted;

	// class

	/**
	 * Called when a SysId test is supposed to have been started. If it is not,
	 * the
	 * user is warned and the application exits.
	 * 
	 * @param testName
	 *            Name of the test.
	 * @param test
	 *            The test command.
	 * @param isStarted
	 *            True if the test was started (no way to tell from the
	 *            command).
	 */
	public static void assureTestStarted(String testName, Command test,
			boolean isStarted) {
		if (!isStarted) {
			CommandScheduler.getInstance().cancel(test); // just in case

			JOptionPane.showMessageDialog(null,
					"Test [" + testName + "] did not start.\n\n" +
							"The SysId log is incomplete.\n\n" +
							"Re-run the application to start over.",
					"SysIdRobot", JOptionPane.ERROR_MESSAGE);

			System.exit(0);
		}
	}

	/**
	 * Called when a SysId test is supposed to be finished. If it is not, the
	 * user is warned and the application exits.
	 * 
	 * @param testName
	 *            Name of the test.
	 * @param test
	 *            The test command.
	 */
	public static void assureTestFinished(String testName, Command test) {
		if (!test.isFinished()) {
			CommandScheduler.getInstance().cancel(test);

			JOptionPane.showMessageDialog(null,
					"Test [" + testName + "] did not finish.\n\n" +
							"The SysId log is incomplete.\n\n" +
							"Re-run the application to start over.",
					"SysIdRobot", JOptionPane.ERROR_MESSAGE);

			System.exit(0);
		}
	}

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
}
