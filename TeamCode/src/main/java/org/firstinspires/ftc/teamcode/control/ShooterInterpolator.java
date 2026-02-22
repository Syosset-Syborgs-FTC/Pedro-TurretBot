package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.List;

public class ShooterInterpolator {
	public static ShooterInterpolator INSTANCE = null; // todo: fill in calibration points
	private final CalibrationPoint[] points;
	private final int kNeighbors;
	private final double power;

	private final double[] distancesSq;
	private final int[] indices;
	private ShooterInterpolator(CalibrationPoint[] points, int kNeighbors, double power) {
		if (points == null || points.length == 0) {
			throw new IllegalArgumentException("You must provide at least one calibration point.");
		}
		this.points = points;
		this.kNeighbors = Math.min(kNeighbors, points.length);
		this.power = power;

		this.distancesSq = new double[points.length];
		this.indices = new int[points.length];
	}


	public ShooterState getTargetState(Pose currentPose) {
		return getTargetState(currentPose, new ShooterState(0, 0));
	}

	public ShooterState getTargetState(Pose currentPose, ShooterState stateToUpdate) {
		if (points.length == 1) {
			stateToUpdate.velocity = points[0].state.velocity;
			stateToUpdate.hoodAngle = points[0].state.hoodAngle;
			return stateToUpdate;
		}

		double x = currentPose.getX();
		double y = currentPose.getY();

		for (int i = 0; i < points.length; i++) {
			double dx = x - points[i].pose.getX();
			double dy = y - points[i].pose.getY();
			double distSq = (dx * dx) + (dy * dy);

			// exact match
			if (distSq < 0.001) {
				stateToUpdate.velocity = points[i].state.velocity;
				stateToUpdate.hoodAngle = points[i].state.hoodAngle;
				return stateToUpdate;
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
			stateToUpdate.velocity = points[indices[0]].state.velocity;
			stateToUpdate.hoodAngle = points[indices[0]].state.hoodAngle;
		} else {
			stateToUpdate.velocity = powerSum / weightSum;
			stateToUpdate.hoodAngle = angleSum / weightSum;
		}

		return stateToUpdate;
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