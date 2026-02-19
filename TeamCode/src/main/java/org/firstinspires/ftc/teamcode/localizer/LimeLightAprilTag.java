package org.firstinspires.ftc.teamcode.localizer;

import android.util.Pair;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Common;

import java.util.Arrays;
import java.util.Optional;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimeLightAprilTag implements Subsystem {
	public static final LimeLightAprilTag INSTANCE = new LimeLightAprilTag();
	Limelight3A limelight;

	public void initialize() {
		limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(60);
		limelight.start();
	}

	public double turretAngle;

	public void setTurretAngle(double turretAngle) {
		this.turretAngle = turretAngle;
	}


	// robot centric inches
	static double turretOffsetX = 0; // forward
	static double turretOffsetY = 0; // left

	public static Pose flattenPose3DTo2d(Pose3D pose3D) {
		Position p = pose3D.getPosition();
		DistanceUnit pUnit = p.unit;
		double x = p.x;
		double y = p.y;

		// extract yaw heading (radians)
		double heading = pose3D.getOrientation().getYaw(AngleUnit.RADIANS);
		return PoseConverter.pose2DToPose(new Pose2D(pUnit, x, y, AngleUnit.RADIANS, heading), InvertedFTCCoordinates.INSTANCE);
	}

	public Pose calculateRobotPoseFromCameraPose(Pose cameraPose) {
		double turretCenterX = cameraPose.getX();
		double turretCenterY = cameraPose.getY();
		double robotHeading = cameraPose.getHeading() + turretAngle;

		// TODO: limelight must be offset such that the returned position is on the axis of rotation
		double rotatedOffsetX = (turretOffsetX * Math.cos(robotHeading)) - (turretOffsetY * Math.sin(robotHeading));
		double rotatedOffsetY = (turretOffsetX * Math.sin(robotHeading)) + (turretOffsetY * Math.cos(robotHeading));

		double robotX = turretCenterX - rotatedOffsetX;
		double robotY = turretCenterY - rotatedOffsetY;
		return new Pose(robotX, robotY, robotHeading, InvertedFTCCoordinates.INSTANCE);
	}

	public void stop() {
		limelight.stop();
	}

	public Optional<Pair<Pose, Long>> localizeRobotMT2() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			return Optional.of(Pair.create((flattenPose3DTo2d(result.getBotpose_MT2())), result.getControlHubTimeStampNanos()));
		}
		return Optional.empty();
	}

	public Optional<Pair<Pose, Long>> localizeRobotMT1() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			Pose3D pose = result.getBotpose();
			ActiveOpMode.telemetry().addData("MT1 std dev", Arrays.toString(result.getStddevMt1()));
			return Optional.of(Pair.create((flattenPose3DTo2d(pose)), result.getControlHubTimeStampNanos()));
		}
		return Optional.empty();
	}

	public void updateRobotOrientation(double heading) {
		limelight.updateRobotOrientation(Math.toDegrees(heading));
	}
}
