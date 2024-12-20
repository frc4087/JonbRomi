package frc.jonb.sysid;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SysIdDrivable extends Subsystem {
	/**
	 * Sets the drive voltage of all motors.
	 * @param volts [-12V, +12V]]
	 */
	public void setVoltage(Voltage volts);

	/**
	 * Logs the current drive state.
	 * @param log Temp input logger.
	 */
	public void logEntry(SysIdRoutineLog log);
}
