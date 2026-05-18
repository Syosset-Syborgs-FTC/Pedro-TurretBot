package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueClose extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(-54.000, -54.000, Math.toRadians(180)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
		Shooter.INSTANCE.setTargetVelocity(1400);
		TurretAngleControl.INSTANCE.setAnglerPosition(0);
		Paths paths = new Paths(follower());
//		Shooter.INSTANCE.startIntake();

//		addIntakeCallbacks(paths.GPP, paths.GPPReturn);
//		addIntakeCallbacks(paths.PGP, paths.PGPReturn);
//		addIntakeCallbacks(paths.PPG, paths.PPGReturn);


		new SequentialGroup(
//				new FollowPath(paths.Preload),
				new WaitUntil(() -> Math.abs(Shooter.INSTANCE.getCurrentVelocity() - Shooter.INSTANCE.getTargetVelocity()) < 15),
//				new FollowPath(paths.LeaveZone),
				Shooter.INSTANCE.shootCommand()
//				new InstantCommand(Shooter.INSTANCE::startIntake),
//				new FollowPath(paths.GPP),
//				new FollowPath(paths.GPPReturn),
//				Shooter.INSTANCE.shootCommand(),
//
//				new FollowPath(paths.PGP),
//				new FollowPath(paths.PGPReturn),
//				Shooter.INSTANCE.shootCommand(),
//
//				new FollowPath(paths.PPG),
//				new InstantCommand(() -> Shooter.INSTANCE.setTargetVelocity(1500)),
//				new FollowPath(paths.PPGReturn),
//				Shooter.INSTANCE.shootCommand()

		).schedule();
	}

	public static class Paths {
		public PathChain Preload;
		public PathChain GPP;
		public PathChain GPPReturn;
		public PathChain PGP;
		public PathChain PGPReturn;
		public PathChain PPG;
		public PathChain PPGReturn;
		public PathChain LeaveZone;

		public Paths(Follower follower) {
			Preload = follower.pathBuilder()
					.addPath(
							new BezierLine(
									follower.getPose(),
									new Pose(-9, -13)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-140))
//					.setBrakingStart()
					.build();

			GPP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-9, -13),
									new Pose(5.045, -12.257),
									new Pose(24.602, -16.382),
									new Pose(26.319, -18.985),
									new Pose(37.761, -22.482)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-90))
					.addPath(
							new BezierLine(
									new Pose(37.761, -22.482),
									new Pose(37.761, -60.482)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(-90))
					.build();

			GPPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(37.761, -60.482),
									new Pose(37.418, -15.757),
									new Pose(-9, -13)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-140))
					.build();

			PGP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-9, -13),
									new Pose(18.843, -12.264),
									new Pose(18.505, -20.730),
									new Pose(18.051, -26.464)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-90))
					.addPath(
							new BezierLine(
									new Pose(18.051, -26.464),
									new Pose(18.051, -60.464)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(-90))
					.build();

			PGPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(18.051, -60.464),
									new Pose(10.290, -24.486),
									new Pose(-9, -13)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-140))
					.build();

			PPG = follower.pathBuilder()
					.addPath(new BezierPoint(-9, -13))
					.setLinearHeadingInterpolation(-140, -90)
					.addPath(
							new BezierCurve(
									new Pose(-9, -13),
									new Pose(-9, -25.032),
									new Pose(-9, -26.502)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-90), 0.3)
					.addPath(
							new BezierLine(
									new Pose(-9, -26.502),
									new Pose(-9, -50.502)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(-90))
					.build();

			PPGReturn = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-9, -50.502),
									new Pose(-38, -10)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-115))
					.build();

			LeaveZone = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-54.000, -54.000),
									new Pose(-60, -35)
//									new Pose(1.134, -38.270)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-90))
					.build();
		}
	}
}
