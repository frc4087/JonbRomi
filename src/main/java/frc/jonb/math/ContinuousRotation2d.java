package frc.jonb.math;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Accepts a Rotation2d source and maintains a continuous rotation angle.
 */
public class ContinuousRotation2d {
	public ContinuousRotation2d(Supplier<Rotation2d> source) {
		_source = source;
		resetState();
	}

	/**
	 * Resets the continuous angle and delta to zero.
	 */
	public void reset() {
		resetState();
	}

	/**
	 * Gets the current continuous angle.
	 * 
	 * @return The result.
	 */
	public Rotation2d get() {
		updateState();
		_isDeltaNew = true;

		return Rotation2d.fromRadians(_angleRad);
	}

	/**
	 * Gets the current delta angle, relative to the previous call to get() or
	 * getDelta(). Assumes the angle change is less than +/-180 deg.
	 * 
	 * @return The result [-180, +180].
	 */
	public Rotation2d getDelta() {
		if (!_isDeltaNew) {
			updateState();
		}
		_isDeltaNew = false;

		return Rotation2d.fromRadians(_angleRadDelta);
	}

	// personal

	private void resetState() {
		_angleRad = 0.0;
		_sampleRadOld = 0.0;
		_angleRadDelta = 0.0;
		_isDeltaNew = false;
	}

	private void updateState() {
		// get new angle [-180, +180]
		double sampleRadNew = _source.get().getRadians();

		// resolve delta (<=180deg)
		_angleRadDelta = sampleRadNew - _sampleRadOld;
		if (_angleRadDelta > Math.PI) {
			_angleRadDelta -= 2 * Math.PI;
		} else if (_angleRadDelta < -Math.PI) {
			_angleRadDelta += 2 * Math.PI;
		}

		_sampleRadOld = sampleRadNew;

		// resolve angle
		_angleRad += _angleRadDelta;
	}

	private Supplier<Rotation2d> _source;
	private double _angleRad;
	private double _sampleRadOld;
	private double _angleRadDelta;
	private boolean _isDeltaNew;
}
