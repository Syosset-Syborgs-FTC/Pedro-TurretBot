package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.OptionalInt;

import dev.nextftc.hardware.positionable.Positionable;
import dev.nextftc.hardware.powerable.Powerable;
@Configurable
public class Common {
	@NonNull
	public static String formatObeliskID(int x) {
		if (x != -1) {
			switch (x) {
				case 21:
					return "GPP";
				case 22:
					return "PGP";
				case 23:
					return "PPG";
				default:
					return Integer.toString(x);
			}
		}
		return "No obelisk apriltag visible";
	}

	public enum Alliance {
		Red,
		Blue;

		public Alliance getOpposite() {
			return this == Red ? Blue : Red;
		}
	}
	public static Alliance alliance = Alliance.Red;
	public static Pose applyTransform(Pose base, Pose offset) {
		Pose rotatedOffset = offset.rotate(base.getHeading(), false);

		return new Pose(
				base.getX() + rotatedOffset.getX(),
				base.getY() + rotatedOffset.getY(),
				MathFunctions.normalizeAngle(base.getHeading() + offset.getHeading()),
				base.getCoordinateSystem()
		);
	}

	public static Pose inverse(Pose pose) {
		double invHeading = -pose.getHeading();

		Pose inverted = new Pose(-pose.getX(), -pose.getY(), invHeading, pose.getCoordinateSystem())
				.rotate(invHeading, false);

		return inverted.withHeading(MathFunctions.normalizeAngle(invHeading));
	}
	public static void overrideCacheZero(Positionable... l) {
		for (Positionable p : l) {
			p.setPosition(0.03);
			p.setPosition(0);
		}
	}
	public static void overrideCacheZero(Powerable... l) {
		for (Powerable p : l) {
			p.setPower(0.02);
			p.setPower(0);
		}
	}
}
