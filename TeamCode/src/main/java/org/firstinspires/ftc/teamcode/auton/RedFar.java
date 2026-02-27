package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class RedFar extends SyborgsAutonBase {
	@Override
	public void onStartButtonPressed() {
		Shooter.INSTANCE.setTargetVelocity(1500);
		TurretAngleControl.INSTANCE.setAnglerPosition(0.5);
		Paths paths = new Paths(follower());

		addIntakeCallbacks(paths.GPP, paths.GPPReturn);
		addIntakeCallbacks(paths.PGP, paths.PGPReturn);
		addIntakeCallbacks(paths.PPG, paths.PPGReturn);

		new SequentialGroup(
				new FollowPath(paths.Preload),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(paths.GPP),
				new FollowPath(paths.GPPReturn),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(paths.PGP),
				new FollowPath(paths.PGPReturn),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(paths.PPG),
				new FollowPath(paths.PPGReturn),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(paths.LeaveZone)
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
							new BezierCurve(
									new Pose(64.000, 16.000),
									new Pose(-7.971, 11.693),
									new Pose(-9, 10)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
					.build();

			GPP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-9, 10),
									new Pose(5.045, 18.257),
									new Pose(24.602, 21.382),
									new Pose(42.319, 19.985),
									new Pose(39.761, 53.482)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			GPPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(39.761, 53.482),
									new Pose(36.418, 15.757),
									new Pose(-9, 10)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			PGP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-9, 10),
									new Pose(13.843, 12.264),
									new Pose(15.505, 30.730),
									new Pose(14.051, 52.464)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			PGPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(14.051, 52.464),
									new Pose(10.290, 24.486),
									new Pose(-9, 10)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			PPG = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-9, 10),
									new Pose(-8.977, 25.032),
									new Pose(-11.936, 45.502)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			PPGReturn = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-11.936, 45.502),
									new Pose(-9, 10)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			LeaveZone = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-9, 10),
									new Pose(1.134, 42.270)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();
		}
	}
}
