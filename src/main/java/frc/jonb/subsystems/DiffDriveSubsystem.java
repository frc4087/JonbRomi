package frc.jonb.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.jonb.pathplanner.PPDrivable;

/**
 * A differential drive Subsystem that supports PathPlanner.
 */
public class DiffDriveSubsystem extends SubsystemBase implements PPDrivable {
	/**
	 * Creates an instance.
	 * 
	 * @param target
	 *            The differential drive target.
	 */
	public DiffDriveSubsystem(DiffDrivable target) {
		ErrorMessages.requireNonNullParam(target, "proxy",
				"DiffDriveSubsystem");
		_target = target;

		_subsystems.add(this);
		Collections.addAll(_subsystems, _target.getSubsystems());

		_kinematics = new DifferentialDriveKinematics(
				_target.getTrackWidth());
		_odometry = new DifferentialDriveOdometry(
				new Rotation2d(), 0.0, 0.0);

		_diffDrive = new DifferentialDrive(
				_leftMotor::set, _rightMotor::set);

	}

	/**
	 * Sets robot chassis speed factors.
	 *
	 * @param forwardFactor
	 *            Forward speed factor [-1,+1].
	 * @param rotateSpeed
	 *            Rotation CCW speed factor [-1,+1].
	 */
	void arcadeDrive(double forwardFactor, double ccwFactor) {
		_diffDrive.arcadeDrive(forwardFactor, ccwFactor);
	}

	/**
	 * Sets robot wheel speed factors.
	 *
	 * @param leftFactor
	 *            Left wheel forward speed factor [-1,+1].
	 * @param rightFactor
	 *            Right wheel forward speed factor [-1,+1].
	 */
	void tankDrive(double leftFactor, double rightFactor) {
		_diffDrive.tankDrive(leftFactor, rightFactor, true);
	}

	// PPDrivable

	@Override
	public Pose2d getPose() {
		return _odometry.getPoseMeters();
	}

	@Override
	public void resetPose(Pose2d pose) {
		_target.resetGyro();
		_target.resetEncoders();

		_odometry.resetPosition(_target.getRotationZ(),
				_target.getLeftDistance(),
				_target.getRightDistance(), pose);
	}

	@Override
	public ChassisSpeeds getTrueSpeeds() {
		DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
				_target.getLeftVelocity(), _target.getRightVelocity());
		return _kinematics.toChassisSpeeds(wheelSpeeds);
	}

	@Override
	public void setDriveSpeeds(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = _kinematics
				.toWheelSpeeds(speeds);
		setWheelSpeeds(wheelSpeeds);
	}

	@Override
	public boolean isHolonomic() {
		return false;
	}

	@Override
	public Subsystem[] getSubsystems() {
		return _subsystems.toArray(Subsystem[]::new);
	}

	// Subsystem

	@Override
	public void periodic() {
		super.periodic();
		_target.periodic();

		_odometry.update(_target.getRotationZ(), _target.getLeftDistance(),
				_target.getRightDistance());
	}

	// personal

	private DiffDrivable _target;
	private List<Subsystem> _subsystems = new ArrayList<>();

	private final DifferentialDriveKinematics _kinematics;
	private final DifferentialDriveOdometry _odometry;
	private final DifferentialDrive _diffDrive;

	private void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
		// WHEEL_MPS_MAX is based on lots of assumptions
		// (i.e. no load, constant 4.5V, etc.)
		double leftFactor = speeds.leftMetersPerSecond
				/ _target.getWheelVelocityMax();
		double rightFactor = speeds.rightMetersPerSecond
				/ _target.getWheelVelocityMax();

		// Without PID, need to reduce motor gain and clamp range.
		double gain = 0.066;
		leftFactor *= gain;
		rightFactor *= gain;

		double maxFactor = 1.0;
		leftFactor = Math.max(-maxFactor,
				Math.min(leftFactor, maxFactor));
		rightFactor = Math.max(-maxFactor,
				Math.min(rightFactor, maxFactor));

		_target.tankDrive(leftFactor, rightFactor);
	}
}
