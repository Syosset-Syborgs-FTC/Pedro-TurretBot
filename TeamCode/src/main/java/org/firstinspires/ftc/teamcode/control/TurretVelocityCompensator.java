package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.ftc.ActiveOpMode;

public class TurretVelocityCompensator implements FeedforwardElement {
	public static TurretVelocityCompensator INSTANCE = new TurretVelocityCompensator();

	public static double maxTurretSpeed = (312.0 / 4.0 / 60.0) * 2.0 * Math.PI;

	public static double kV = 1.0 / maxTurretSpeed;
	public static double kA = 0.01;

	@Override
	public double calculate(@NonNull KineticState referenceState) {
		double profiledVel = referenceState.getVelocity();
		double profiledAccel = referenceState.getAcceleration();

		double headingVel = SensorFusion.INSTANCE.getVelocity().getHeading();

		double targetTotalVel = profiledVel - headingVel;

		double ff = (kV * targetTotalVel) + (kA * profiledAccel);

		double currentPos = TurretAngleControl.INSTANCE.getActualTurretAngle();

		boolean nearNegStop = currentPos < Math.toRadians(-200);
		boolean nearPosStop = currentPos > Math.toRadians(130);

		// cut power if driving towards hard stop
		if ((nearNegStop && targetTotalVel < 0) || (nearPosStop && targetTotalVel > 0)) {
			ff = 0;
		}
		if (Math.abs(headingVel) < 0.01) ff = 0;

		ActiveOpMode.telemetry().addData("Turret FF target vel", targetTotalVel);
		ActiveOpMode.telemetry().addData("Turret near hard stop", nearNegStop || nearPosStop);

		return 0;
	}

	@Override
	public void reset() {}
}
