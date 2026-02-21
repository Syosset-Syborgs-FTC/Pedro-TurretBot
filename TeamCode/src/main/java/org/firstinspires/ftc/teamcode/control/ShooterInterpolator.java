package org.firstinspires.ftc.teamcode.control;


import com.pedropathing.geometry.Pose;
public class ShooterInterpolator {
	public static ShooterInterpolator INSTANCE = new ShooterInterpolator(
//			new CalibrationPoint(),
//			new CalibrationPoint()
	);
	private final CalibrationPoint[] points;
	public ShooterInterpolator(CalibrationPoint... points) {
		if (points == null || points.length == 0) {
			throw new IllegalArgumentException("You must provide at least one calibration point.");
		}
		this.points = points;
	}


	public ShooterState getTargetState(Pose currentPose) {
		if (points.length == 1) {
			return new ShooterState(points[0].state.power, points[0].state.hoodAngle);
		}

		double x = currentPose.getX();
		double y = currentPose.getY();

		double weightSum = 0;
		double powerSum = 0;
		double angleSum = 0;

		for (CalibrationPoint pt : points) {
			double dx = x - pt.pose.getX();
			double dy = y - pt.pose.getY();

			double distanceSquared = (dx * dx) + (dy * dy);

			if (distanceSquared < 0.001) {
				return new ShooterState(pt.state.power, pt.state.hoodAngle);
			}

			double weight = 1.0 / distanceSquared;
			weightSum += weight;

			powerSum += pt.state.power * weight;
			angleSum += pt.state.hoodAngle * weight;
		}

		return new ShooterState(powerSum / weightSum, angleSum / weightSum);
	}
	public static class CalibrationPoint {
		public Pose pose;
		public ShooterState state;

		public CalibrationPoint(Pose pose, ShooterState state) {
			this.pose = pose;
			this.state = state;
		}
	}
	public static class ShooterState {
		public double power;
		public double hoodAngle;

		public ShooterState(double power, double hoodAngle) {
			this.power = power;
			this.hoodAngle = hoodAngle;
		}
	}
}
