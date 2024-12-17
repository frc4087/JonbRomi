package frc.jonb.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Represents a bridge between PathPlanner and a PPDrivable subsystem.
 * To connect PathPlanner to a robot's drivetrain, call a "builder" method.
 */
public class PPBridge<T extends PPDrivable> {
  /**
   * Creates an instance. For extension.
   */
  protected PPBridge(T proxy) {
    _proxy = proxy;

    // resolve PP config from PP GUI settings
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      throw new IllegalStateException(
          "Robot configuration not found in GUI settings.");
    }

    // connect PP to drivetrain
    AutoBuilder.configure(
        _proxy::getPose,
        _proxy::resetPose,
        _proxy::getTrueSpeeds,
        (speeds, feedforwards) -> _proxy.setDriveSpeeds(speeds),
        new PPLTVController(0.02),
        config,
        PPBridge::isPathFlipped,
        _proxy.getSubsystems());
  }

  public T getProxy() {
    return _proxy;
  }

  // personal

  private T _proxy;

  // class

  /**
   * Establishes a bridge between PathPlannerf and a given drivetrain subsystem,
   * and returns the bridge object, if needed.
   * 
   * @param proxy
   *        PathPlanner drivable proxy.
   */
  public static <T extends PPDrivable> PPBridge<T>  buildBridge(T proxy) {
    return new PPBridge<T>(proxy);
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
}
