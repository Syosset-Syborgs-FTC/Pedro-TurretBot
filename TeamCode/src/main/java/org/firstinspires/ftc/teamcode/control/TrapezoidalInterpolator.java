package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.interpolators.InterpolatorElement;
import dev.nextftc.ftc.ActiveOpMode;

public class TrapezoidalInterpolator implements InterpolatorElement {
	private final double maxVel;
	private final double maxAccel;

	private KineticState startState = new KineticState();
	private KineticState goalState = new KineticState();
	private KineticState currentReference = new KineticState();

	private final ElapsedTime timer = new ElapsedTime();

	private double dir, tAccel, tCruise, tDecel, totalTime;
	private double dAccel, dCruise, peakVel;

	public TrapezoidalInterpolator(double maxVel, double maxAccel) {
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		this.timer.reset();
	}

	@NonNull
	@Override
	public KineticState getGoal() {
		return goalState;
	}

	@Override
	public void setGoal(@NonNull KineticState goal) {
		// regenerate if changed
		if (Math.abs(goal.getPosition() - goalState.getPosition()) > 1e-4) {
			startState = getCurrentReference(); // start from current state
			goalState = goal;
			computeProfile();
		}
	}

	private void computeProfile() {
		double dist = goalState.getPosition() - startState.getPosition();
		dir = Math.signum(dist);
		dist = Math.abs(dist);

		tAccel = maxVel / maxAccel;
		dAccel = 0.5 * maxAccel * tAccel * tAccel;

		if (dist > 2 * dAccel) {
			// reaches max speed (trapezoid)
			dCruise = dist - 2 * dAccel;
			tCruise = dCruise / maxVel;
			tDecel = tAccel;
			peakVel = maxVel;
		} else {
			// doesn't reach max speed (triangle)
			dAccel = dist / 2.0;
			tAccel = Math.sqrt(2.0 * dAccel / maxAccel);
			tCruise = 0;
			dCruise = 0;
			tDecel = tAccel;
			peakVel = maxAccel * tAccel;
		}
		totalTime = tAccel + tCruise + tDecel;
		timer.reset();
	}

	@NonNull
	@Override
	public KineticState getCurrentReference() {
		double t = timer.seconds();

		// done
		if (t >= totalTime) {
			currentReference = new KineticState(goalState.getPosition(), 0.0, 0.0);
			return currentReference;
		}

		double p = startState.getPosition();
		double v, a;

		if (t < tAccel) {
			// accelerating
			a = maxAccel * dir;
			v = a * t;
			p += 0.5 * a * t * t;
		} else if (t < tAccel + tCruise) {
			// cruising
			double tC = t - tAccel;
			a = 0;
			v = peakVel * dir;
			p += (dAccel * dir) + v * tC;
		} else {
			// decelerating
			double tD = t - tAccel - tCruise;
			a = -maxAccel * dir;
			v = (peakVel * dir) + a * tD;
			p += ((dAccel + dCruise) * dir) + (peakVel * dir * tD) + 0.5 * a * tD * tD;
		}

		currentReference = new KineticState(p, v, a);
		return currentReference;
	}

	@Override
	public void reset() {
		startState = new KineticState();
		goalState = new KineticState();
		currentReference = new KineticState();
		timer.reset();
	}
}
