package frc.jonb.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Represents a bridge between PathPlanner and a PPDrivable subsystem.
 * To connect PathPlanner to a robot's drivetrain, call a "builder" method.
 */
public class PPBridge<T extends PPDrivable> {
	/**
	 * For extension.
	 * 
	 * @param drive
	 *            target drivetrain.
	 */
	protected PPBridge(T drive) {
		ErrorMessages.requireNonNullParam(drive, "drive", "PPBridge");
		_drive = drive;

		// resolve PP config from PP GUI settings
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			throw new IllegalStateException(
					"Robot configuration not found in GUI settings.");
		}

		// builkd PP controller
		//// PPLTVController controller = new PPLTVController(0.02, 1.0);
		//// PPLTVController controller = new PPLTVController(
		//// VecBuilder.fill(0.05, 0.05, 0.05), VecBuilder.fill(0.1, 0.1),
		//// 0.02);
		PathFollowingController controller = new MyController();

		// connect PP to drivetrain
		AutoBuilder.configure(
				_drive::getPose,
				_drive::resetPose,
				_drive::getTrueSpeeds,
				(speeds, feedforwards) -> _drive.setDriveSpeeds(speeds),
				controller,
				config,
				PPBridge::isPathFlipped,
				_drive.getSubsystems());
	}

	/**
	 * Gets the target drivetrain.
	 * 
	 * @return Shared exposed drivetrain.
	 */
	public T getDrive() {
		return _drive;
	}

	// personal

	private T _drive;

	// class

	/**
	 * Establishes a bridge between PathPlannerf and a given drivetrain
	 * subsystem,
	 * and returns the bridge object, if needed.
	 * 
	 * @param drive
	 *            Target drive train.
	 */
	public static <T extends PPDrivable> PPBridge<T> buildBridge(T drive) {
		return new PPBridge<T>(drive);
	}

	/**
	 * Returns true if the path being followed should be mirrored (i.e. for the
	 * red alliance)
	 * 
	 * @return The state.
	 */
	public static boolean isPathFlipped() {
		var alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			return alliance.get() == DriverStation.Alliance.Red;
		}
		return false;
	}

	public  class MyController implements PathFollowingController {

		@Override
		public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose,
				PathPlannerTrajectoryState targetState) {
			Pose2d poseError = targetState.pose.relativeTo(currentPose);
			double forwardMps = poseError.getX() * 1.0;
			double turnRps = poseError.getRotation().getRadians() * 1.0;
			// System.out.printf(
			// 		"MyController: z=%6.1f errXYZ=%5.2f %5.2f %6.1f velXZ=%6.3f %6.3f\n",
			// 		_drive.getPose().getRotation().getDegrees(),
			// 		//currentPose.getRotation().getDegrees(),
			// 		poseError.getX(),
			// 		poseError.getY(), poseError.getRotation().getDegrees(),
			// 		forwardMps, turnRps * 2.0 * Math.PI);
			return new ChassisSpeeds(forwardMps, 0.0, turnRps);
		}

		@Override
		public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
			// do nothing
		}

		@Override
		public boolean isHolonomic() {
			return false;
		}
	}
}
