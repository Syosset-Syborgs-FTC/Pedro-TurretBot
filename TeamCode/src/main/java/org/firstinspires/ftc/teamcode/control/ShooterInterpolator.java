package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;

public class ShooterInterpolator {
	public static ShooterInterpolator INSTANCE = new ShooterInterpolator(5, 2);
	private CalibrationPoint[] points = {
			new CalibrationPoint(
					new Pose(-53, 54),
					new ShooterState(1400, 0)
			),
			new CalibrationPoint(
					new Pose(-48, 38),
					new ShooterState(1400, 0.02)
			),
			new CalibrationPoint(
					new Pose(-3, 18),
					new ShooterState(1530, 0.25)
			),
			new CalibrationPoint(
					new Pose(-65, 10),
					new ShooterState(1500, 0.25)
			),
			new CalibrationPoint(
					new Pose(-65, -1),
					new ShooterState(1550, 0.35)
			),
			new CalibrationPoint(
					new Pose(-14.86, 10.06),
					new ShooterState(1650, 0.45)
			),
			new CalibrationPoint(
					new Pose(-32.5, -13.1),
					new ShooterState(1620, 0.55)
			),
			new CalibrationPoint(
					new Pose(-22.07, -33.6),
					new ShooterState(1250, 0.7)
			),
			new CalibrationPoint(
					new Pose(0,0),
					new ShooterState(1700, 0.65)
			),
			new CalibrationPoint(
					new Pose(-49, -23),
					new ShooterState(1700, 0.65)
			),
			new CalibrationPoint(
					new Pose(-65, -36),
					new ShooterState(1800, 0.75)
			),
	};
	private final int kNeighbors;
	private final double power;

	private final double[] distancesSq;
	private final int[] indices;

	private ShooterInterpolator(int kNeighbors, double power) {
		this.kNeighbors = Math.min(kNeighbors, points.length);
		this.power = power;

		this.distancesSq = new double[points.length];
		this.indices = new int[points.length];
	}



	public ShooterState getTargetState(Pose currentPose) {
		ShooterState state = new ShooterState(0,0);
		if (points.length == 1) {
			state.velocity = points[0].state.velocity;
			state.hoodAngle = points[0].state.hoodAngle;
			return state;
		}

		double x = currentPose.getX();
		double y = currentPose.getY();

		for (int i = 0; i < points.length; i++) {
			double dx = x - points[i].pose.getX();
			double dy = y - points[i].pose.getY();
			double distSq = (dx * dx) + (dy * dy);

			// exact match
			if (distSq < 0.001) {
				state.velocity = points[i].state.velocity;
				state.hoodAngle = points[i].state.hoodAngle;
				return state;
			}

			distancesSq[i] = distSq;
			indices[i] = i;
		}

		// k nearest neighbors
		for (int i = 0; i < kNeighbors; i++) {
			int minIdx = i;
			for (int j = i + 1; j < points.length; j++) {
				if (distancesSq[indices[j]] < distancesSq[indices[minIdx]]) {
					minIdx = j;
				}
			}
			// Swap
			int temp = indices[i];
			indices[i] = indices[minIdx];
			indices[minIdx] = temp;
		}

		// run idw
		double weightSum = 0;
		double powerSum = 0;
		double angleSum = 0;

		for (int i = 0; i < kNeighbors; i++) {
			int ptIndex = indices[i];
			double distSq = distancesSq[ptIndex];

			double weight;
			if (Math.abs(power - 2.0) < 0.001) {
				weight = 1.0 / distSq;
			} else {
				weight = 1.0 / Math.pow(distSq, power / 2.0);
			}

			weightSum += weight;
			powerSum += points[ptIndex].state.velocity * weight;
			angleSum += points[ptIndex].state.hoodAngle * weight;
		}

		// prevent div by 0
		if (weightSum == 0) {
			state.velocity = points[indices[0]].state.velocity;
			state.hoodAngle = points[indices[0]].state.hoodAngle;
		} else {
			state.velocity = powerSum / weightSum;
			state.hoodAngle = angleSum / weightSum;
		}

		return state;
	}

	public static class CalibrationPoint {
		public final Pose pose;
		public final ShooterState state;

		public CalibrationPoint(Pose pose, ShooterState state) {
			this.pose = pose;
			this.state = state;
		}
	}

	public static class ShooterState {
		public double velocity;
		public double hoodAngle;

		public ShooterState(double velocity, double hoodAngle) {
			this.velocity = velocity;
			this.hoodAngle = hoodAngle;
		}
	}
}