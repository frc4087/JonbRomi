package frc.jonb.sysid;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Test command that drives the robot forward at a "slow" speed until the
 * end() is called. Used to test robot motor activation and button
 * bindings for running SysId tests.
 */
public class SysIdTestCommand extends Command {
	public SysIdTestCommand(SysIdDrivable drive) {
		_drive = drive;
		addRequirements(_drive);
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}

	@Override
	public void initialize() {
		System.out.println("Starting SysIdTestCommand");
	}

	@Override
	public void execute() {
		_drive.setVoltage(Units.Volts.of(2.0));
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Stopping SysIdTestCommand");
		_drive.setVoltage(Units.Volts.of(0.0));
	}

	private SysIdDrivable _drive;
}
