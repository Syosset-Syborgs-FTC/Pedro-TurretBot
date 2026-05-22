package org.firstinspires.ftc.teamcode.localizer;

import android.util.Pair;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;
import java.util.Optional;

import dev.nextftc.ftc.ActiveOpMode;

public class SensorFusion implements Localizer {
	public static SensorFusion INSTANCE = new SensorFusion();
	Pose startPose;

	PoseFilter filter = new PoseFilter();
	PinpointLocalizer pinpointLocalizer;
	public Optional<Pose> cachedMT1Pose = Optional.of(new Pose());
	public Optional<Pose> cachedMT2Pose = Optional.of(new Pose());

	public Optional<Pose> cachedCameraPose = Optional.empty();

	public void init(HardwareMap hardwareMap, PinpointConstants pinpointConstants) {
		pinpointLocalizer = new PinpointLocalizer(hardwareMap, pinpointConstants, new Pose());
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());
		cachedMT1Pose = Optional.empty();
		cachedMT2Pose = Optional.empty();
		cachedCameraPose = Optional.empty();
		setStartPose(new Pose());
		setPose(new Pose());
	}

	@Override
	public Pose getPose() {
		return filter.getPose(pinpointLocalizer.getPose());
	}

	@Override
	public Pose getVelocity() {
		Pose rawVel = pinpointLocalizer.getVelocity();
		return filter.getVelocity(rawVel);
	}

	@Override
	public Vector getVelocityVector() {
		return getVelocity().getAsVector();
	}


	@Override
	public void setStartPose(Pose setStart) {
		if (!Objects.equals(startPose, new Pose()) && startPose != null) {
			Pose currentPose = getPose().rotate(-startPose.getHeading(), false).minus(startPose);
			setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
		} else {
			setPose(setStart);
		}

		this.startPose = setStart;
	}


	@Override
	public void setPose(Pose setPose) {
		filter.setPose(pinpointLocalizer.getPose(), setPose);
	}


	@Override
	public void update() {
		pinpointLocalizer.update();
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());
		Optional<LimeLightAprilTag.VisionResult> mt1Result = LimeLightAprilTag.INSTANCE
				.localizeRobotMT1();
		ActiveOpMode.telemetry().addData("updated", true);
		mt1Result
				.ifPresent(visionResult -> {
					cachedCameraPose = Optional.of(visionResult.pose);
					filter.updateVision(LimeLightAprilTag.INSTANCE.calculateRobotPoseFromCameraPose(visionResult.pose), visionResult.timestamp, visionResult.llTimestamp, visionResult.stddev);
				});
		cachedMT1Pose = mt1Result.map(p -> p.pose);
		cachedMT2Pose = LimeLightAprilTag.INSTANCE.localizeRobotMT2().map(p -> p.pose);
		LimeLightAprilTag.INSTANCE.updateRobotOrientation(filter.getPose(pinpointLocalizer.getPose()).getHeading());
	}

	@Override
	public double getTotalHeading() {
		return pinpointLocalizer.getTotalHeading();
	}

	@Override
	public double getForwardMultiplier() {
		return pinpointLocalizer.getForwardMultiplier();
	}

	@Override
	public double getLateralMultiplier() {
		return pinpointLocalizer.getLateralMultiplier();
	}

	@Override
	public double getTurningMultiplier() {
		return pinpointLocalizer.getTurningMultiplier();
	}

	@Override
	public void resetIMU() throws InterruptedException {
		pinpointLocalizer.resetIMU();
	}

	@Override
	public double getIMUHeading() {
		return pinpointLocalizer.getIMUHeading();
	}

	@Override
	public boolean isNAN() {
		Pose pose = getPose();
		return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
	}

	public double getRawPinpointHeading() {
		return pinpointLocalizer.getPose().getHeading();
	}
}
