package frc.robot;

import frc.jonb.sysid.SysIdHelper;
import frc.robot.subsystems.RomiDriveSubsystem;

public class RomiSysIdApp {
	public static void main(String... args) {
		RomiDriveSubsystem drive = new RomiDriveSubsystem();
		new SysIdHelper(drive);

		SysIdHelper.showDialogAndWait("RomiSysIdApp");
	}
	
	// /**
	//  * Used as the main class entry in an executable jar, which contains the
	//  * application, its jars, and its native libraries.
	//  * @author jonb
	//  * @see JarClassLoader
	//  */
	// public static class Launcher {
	// 	public static void main(String[] args) {
	// 		JarClassLoader jcl = new JarClassLoader();
	// 		try {
	// 			jcl.invokeMain("frc.robot.RomiSysIdApp", args);
	// 		} catch (Throwable e) {
	// 			e.printStackTrace();
	// 		}
	// 	}
	// }
}
