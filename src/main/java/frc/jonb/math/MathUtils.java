package frc.jonb.math;

public class MathUtils {
	private MathUtils() {}

	/**
	 * Clamps a value to a given range.
	 * @param val Input value.
	 * @param min Range minimum value.
	 * @param max Range maximum value (>=min).
	 * @return The result [min, max].
	 */
	public static double clamp(double val, double min, double max) {
		return Math.max(min, Math.min(max, val));
	}
}
