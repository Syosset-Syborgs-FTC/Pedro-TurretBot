package org.firstinspires.ftc.teamcode.subsytems;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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

	public void periodic() {
		LLResult result = limelight.getLatestResult();
		if (result.isValid()) {
			double turretAngle = 0;
			Pose2D res = calculateRobotPoseFromCameraPose(result.getBotpose(), turretAngle);
			PoseConverter.pose2DToPose(res, InvertedFTCCoordinates.INSTANCE);
		}
	}

	// robot centric inches
	double turretOffsetX = 0; // forward
	double turretOffsetY = 0; // left

	public Pose2D calculateRobotPoseFromCameraPose(Pose3D cameraPose, double turretAngle) {
		Position cameraPosition = cameraPose.getPosition().toUnit(DistanceUnit.INCH);
		double turretCenterX = cameraPosition.x;
		double turretCenterY = cameraPosition.y;
		double robotHeading = cameraPose.getOrientation().getYaw(AngleUnit.RADIANS) + turretAngle;

		// TODO: limelight must be offset such that the returned position is on the axis of rotation
		double rotatedOffsetX = (turretOffsetX * Math.cos(robotHeading)) - (turretOffsetY * Math.sin(robotHeading));
		double rotatedOffsetY = (turretOffsetX * Math.sin(robotHeading)) + (turretOffsetY * Math.cos(robotHeading));

		double robotX = turretCenterX - rotatedOffsetX;
		double robotY = turretCenterY - rotatedOffsetY;
		return new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.RADIANS, robotHeading);
	}
}
