package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for a differential drive subsystem. Methods are tailored for
 * high-level control and status of a differential drive robot, such as 
 * a Romi, and for PathPlanner support.
 */
public interface DiffiDrivable extends Subsystem {

    /**
     * Gets the current robot pose relative to the play field. The pose is
     * updated automatically as the robot moves relative to the starting pose
     * (i.e.
     * the default pose or that set by setPose()).
     * 
     * @return The pose.
     */
    Pose2d getPose();

    /**
     * Resets the current robot pose relative to the play field. The pose should be
     * that of the physical robot (e.g. at the start of automomous mode).
     * 
     * @param pose
     *        The pose.
     */
    void resetPose(Pose2d pose);

    /**
     * Gets the actual robot relative speed (not neccesarily
     * that set by setDriveSpeeds()).
     * 
     * @return The speeds.
     */
    ChassisSpeeds getTrueSpeeds();

    /**
     * Sets the desired robot relative speed.
     * 
     * @param speeds
     *        The speeds.
     */
    void setDriveSpeeds(ChassisSpeeds speeds);

    /**
     * Gets the subsystems required to support this subsystem, including this one.
     * 
     * @return Temp output group.
     */
    List<Subsystem> getSubsystems();

}