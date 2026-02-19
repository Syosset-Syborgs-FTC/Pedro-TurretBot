package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;

public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants().mass(12.7);
	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName("fl")
			.rightRearMotorName("br")
			.rightFrontMotorName("fr")
			.leftRearMotorName("bl")
			.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
	public static PinpointConstants pinpointConstants = new PinpointConstants()
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
			.distanceUnit(DistanceUnit.INCH)
			.forwardPodY(0.7)
			.strafePodX(-0.5);


	public static Follower createFollower(HardwareMap hardwareMap) {
		SensorFusion.INSTANCE.init(hardwareMap, pinpointConstants);
		return new FollowerBuilder(followerConstants, hardwareMap)
				.setLocalizer(SensorFusion.INSTANCE)
				.mecanumDrivetrain(driveConstants)
				.pathConstraints(pathConstraints)
				.build();
	}
}
