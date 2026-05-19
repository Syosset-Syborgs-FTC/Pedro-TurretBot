package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
@Configurable
public class Constants {
	public static FollowerConstants followerConstants = new FollowerConstants()
			.mass(12.7)
			.forwardZeroPowerAcceleration(-33.1)
			.lateralZeroPowerAcceleration(-67.8)
			.translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0.0, 0, 0.01))
			.headingPIDFCoefficients(new PIDFCoefficients(0.5, 0.02, 0.0, 0.03))
			.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0.0000001, 0.6, 0.0))
			.centripetalScaling(0.00094)
			.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.11467282159115731, 0.0014326160024220227));


	public static MecanumConstants driveConstants = new MecanumConstants()
			.maxPower(1)
			.leftFrontMotorName("fl")
			.rightRearMotorName("br")
			.rightFrontMotorName("fr")
			.leftRearMotorName("bl")
			.leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
			.leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
			.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
			.rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
			.xVelocity(78)
			.yVelocity(51);
	public static volatile double forwardPodY = -122.5;
	public static volatile double strafePodX = -192;
	public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.75, 	1);

	public static PinpointConstants pinpointConstants = new PinpointConstants()
			.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
			.distanceUnit(DistanceUnit.MM)
			.forwardPodY(forwardPodY)
			.strafePodX(strafePodX);

	public static Follower createFollower(HardwareMap hardwareMap) {
		SensorFusion.INSTANCE.init(hardwareMap, pinpointConstants);

//		PinpointLocalizer tuningLocalizer = new PinpointLocalizer(hardwareMap, pinpointConstants, new Pose());
		return new FollowerBuilder(followerConstants, hardwareMap)
				.setLocalizer(SensorFusion.INSTANCE)
//				.setLocalizer(tuningLocalizer)
				.mecanumDrivetrain(driveConstants)
				.pathConstraints(pathConstraints)
				.build();
	}
}
