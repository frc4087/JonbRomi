package frc.jonb.sysid;

import javax.swing.JOptionPane;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * A helper class for generating the SysId for a drivetrain subsystem. Binds
 * SysId commands to an XBox controller.
 * <p>
 * To create a SysId log, create an instance of this class with the target
 * drivetrain, then manually execute all of the SysId commands. When done, use
 * the DataLogTool app to analyze the log. See
 * {@link}https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/
 * for details.
 */
public class SysIdHelper {
	/**
	 * Creates an instance.
	 * 
	 * @param drive
	 *        Shared exposed drivetrain subsystem.
	 */
	public SysIdHelper(SysIdDrivable drive) {
		ErrorMessages.requireNonNullParam(drive, "drive", "SysIdHelper");

		_routine = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(drive::setVoltage, drive::logEntry,
						drive));
	}

	/**
	 * Configures a given Xbox controller for triggering SysId routines.
	 * A routine execute only while the corresponding buttons are
	 * held down -- releasing them stops the routine immediately, for safety.
	 * Typically, routines are run in order:
	 * <l>
	 * <li>Quasistatic, forward - A + R-bumper
	 * <li>Quasistatic, backward - B + R-bumper
	 * <li>Dynamic, forward - X + R-bumper
	 * <li>Dynamic, backward - Y + R-bumper
	 * </l>
	 * 
	 * @param hid
	 *        Temp input control device.
	 */
	public void configHid(CommandXboxController hid) {
		hid.a().and(hid.rightBumper()).whileTrue(
				_routine.quasistatic(SysIdRoutine.Direction.kForward));
		hid.b().and(hid.rightBumper()).whileTrue(
				_routine.quasistatic(SysIdRoutine.Direction.kReverse));
		hid.x().and(hid.rightBumper())
				.whileTrue(_routine.dynamic(SysIdRoutine.Direction.kForward));
		hid.y().and(hid.rightBumper())
				.whileTrue(_routine.dynamic(SysIdRoutine.Direction.kReverse));
	}

	// personal

	private final SysIdRoutine _routine;

	// class

	/**
	 * Shows a modal dialog eplaining to the user what to do. Waits until the
	 * user presses the "OK" button.
	 */
	public static void showDialogAndWait(String appName) {
		JOptionPane.showConfirmDialog(null,
				"Place the robot in a clear area with at\n" +
				"least 10 feet of clear space in front of it.\n\n" +
				"Execute each of the following SysId\n" +
				"commands using an XBox controller\n\n"
						+
						"Press OK when done.\n\n" +
						"    A + Rgt-Bumper - Quasistatic forward.\n" +
						"    B + Rgt-Bumper - Quasistatic backward.\n" +
						"    X + Rgt-Bumper - Dynamic forward.\n" +
						"    Y + Rgt-Bumper - Dynamic backward.\n\n",
				appName, JOptionPane.DEFAULT_OPTION);
	}
}
