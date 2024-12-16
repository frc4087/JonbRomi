package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.subsystems.PPDrivable;

/**
 * A PathPlanner drive Subsystem with a DiffiDriveSubsystem proxy.
 */
public class PPDriveSubsystem extends SubsystemBase {
  /**
   * Creates an instance.
   * @param proxy PathPlanner drivable proxy.
   */
  public PPDriveSubsystem(PPDrivable proxy) {
    _proxy = proxy;
    _subsystems.add(this);
    Collections.addAll(_subsystems, _proxy.getSubsystems());

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      throw new IllegalStateException(
          "Robot configuration not found in GUI settings.");
    }

    // Configure AutoBuilder last.
    // Note: The ideal path conforms to max constraints but the generated path
    // may not.
    AutoBuilder.configure(
        _proxy::getPose,
        _proxy::resetPose,
        _proxy::getTrueSpeeds,
        (speeds, feedforwards) -> _proxy.setDriveSpeeds(speeds),
        new PPLTVController(0.02),
        config,
        this::isPathFlipped,
        getSubsystems());
  }

  /**
   * Returns true if the path being followed should be mirrored (i.e. for the
   * red alliance)
   * 
   * @return The state.
   */
  public boolean isPathFlipped() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  /**
   * Gets the subsystems required to support this subsystem, including this one.
   * 
   * @return Temp output group.
   */
  public Subsystem[] getSubsystems() {
    return _subsystems.toArray(Subsystem[]::new);
  }

  @Override
  public void periodic() {
    super.periodic();
    _proxy.periodic();
  }

  // personal

  private PPDrivable _proxy;
  private List<Subsystem> _subsystems = new ArrayList<>();
}
