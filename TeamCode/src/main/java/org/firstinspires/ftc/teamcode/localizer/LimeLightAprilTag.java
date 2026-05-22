package org.firstinspires.ftc.teamcode.localizer;

import android.util.Pair;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import java.util.Arrays;
import java.util.Optional;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimeLightAprilTag implements Subsystem {
	public static final LimeLightAprilTag INSTANCE = new LimeLightAprilTag();
	public static double MAX_CAMERA_OMEGA_DEG_PER_SEC = 60.0;
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
	static double turretOffsetX = -3; // forward
	static double turretOffsetY = -1.5; // left

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

	public Optional<VisionResult> localizeRobotMT2() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			return Optional.of(new VisionResult(flattenPose3DTo2d(result.getBotpose_MT2()), result.getControlHubTimeStampNanos(), result.getTimestamp(), result.getStddevMt2()));
		}
		return Optional.empty();
	}

	public Optional<VisionResult> localizeRobotMT1() {
		double robotOmega = SensorFusion.INSTANCE.getVelocity().getHeading();
		double turretOmega = TurretAngleControl.INSTANCE.turret.getState().getVelocity() * TurretAngleControl.INSTANCE.scalar;
		double totalCameraOmega = Math.abs(robotOmega - turretOmega);

		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			boolean hasGoalTags = false;
			for (FiducialResult fiducial: result.getFiducialResults()) {
				int id = fiducial.getFiducialId();
				if (id == 20 || id == 24) {
					hasGoalTags = true;
					break;
				}
			}
			ActiveOpMode.telemetry().addData("omega", totalCameraOmega);
			if (!hasGoalTags || totalCameraOmega > Math.toRadians(MAX_CAMERA_OMEGA_DEG_PER_SEC)) return Optional.empty();
			Pose3D pose = result.getBotpose();
			ActiveOpMode.telemetry().addData("z", pose.getPosition().z);
			ActiveOpMode.telemetry().addData("MT1 std dev", Arrays.toString(result.getStddevMt1()));
			if (pose.getPosition().z > 0.1) return Optional.empty();
			return Optional.of(new VisionResult(flattenPose3DTo2d(pose), result.getControlHubTimeStampNanos(), result.getTimestamp(), result.getStddevMt1()));
		}
		return Optional.empty();
	}

	public void updateRobotOrientation(double heading) {
		limelight.updateRobotOrientation(Math.toDegrees(heading));
	}
	public static class VisionResult {
		public Pose pose;
		public long timestamp;
		public double llTimestamp;
		public double[] stddev;

		public VisionResult(Pose pose, long timestamp, double llTimestamp, double[] stddev) {
			this.pose = pose;
			this.timestamp = timestamp;
			this.llTimestamp = llTimestamp;
			this.stddev = stddev;
		}
	}
}
