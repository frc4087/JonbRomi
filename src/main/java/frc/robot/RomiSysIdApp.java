package frc.robot;

import frc.jonb.sysid.SysIdHelper;

public class RomiSysIdApp {
	public static void main(String... args) {
		// RomiDriveSubsystem drive = new RomiDriveSubsystem();
		// new SysIdHelper(drive);

		SysIdHelper.showDialogAndWait("RomiSysIdApp");
	}
}
