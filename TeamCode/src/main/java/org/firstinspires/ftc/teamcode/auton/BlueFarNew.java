package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueFarNew extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(64.000, -12.000, Math.toRadians(270)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
		Shooter.INSTANCE.setTargetVelocity(1670);
		TurretAngleControl.INSTANCE.setAnglerPosition(0.6);

		PathChain paths = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(64.000, -12.000),
								new Pose(60.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(290))
				.addPath(
						new BezierLine(
								new Pose(60.000, -62.000),
								new Pose(64.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(290), Math.toRadians(360))
				.addPath(
						new BezierLine(
								new Pose(64.000, -62.000),
								new Pose(60.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(270))
				.addPath(
						new BezierLine(
								new Pose(60.000, -12.000),
								new Pose(37.000, -37.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
				.addPath(
						new BezierLine(
								new Pose(37.000, -37.000),
								new Pose(37.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
				.addPath(
						new BezierLine(
								new Pose(37.000, -62.000),
								new Pose(61.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(270))
				.setReversed()
				.addPath(
						new BezierLine(
								new Pose(61.000, -12.000),
								new Pose(62.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(236))
				.addPath(
						new BezierLine(
								new Pose(62.000, -62.000),
								new Pose(62.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(235))
				.addPath(
						new BezierLine(
								new Pose(62.000, -62.000),
								new Pose(37.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(180))
				.addPath(
						new BezierLine(
								new Pose(37.000, -62.000),
								new Pose(61.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
				.setReversed()
				.addPath(
						new BezierLine(
								new Pose(61.000, -12.000),
								new Pose(62.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(235))
				.addPath(
						new BezierLine(
								new Pose(62.000, -62.000),
								new Pose(37.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(180))
				.addPath(
						new BezierLine(
								new Pose(37.000, -62.000),
								new Pose(61.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
				.addPath(
						new BezierLine(
								new Pose(61.000, -12.000),
								new Pose(62.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(235))
				.addPath(
						new BezierLine(
								new Pose(62.000, -62.000),
								new Pose(37.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(180))
				.addPath(
						new BezierLine(
								new Pose(37.000, -62.000),
								new Pose(61.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
				.addPath(
						new BezierLine(
								new Pose(61.000, -12.000),
								new Pose(62.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(240))
				.addPath(
						new BezierLine(
								new Pose(62.000, -62.000),
								new Pose(37.000, -62.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(180))
				.addPath(
						new BezierLine(
								new Pose(37.000, -62.000),
								new Pose(61.000, -12.000)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
				.build();
		new FollowPath(paths).schedule();
	}
}
