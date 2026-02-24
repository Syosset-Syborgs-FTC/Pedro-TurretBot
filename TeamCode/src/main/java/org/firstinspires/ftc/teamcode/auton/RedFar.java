package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
@Autonomous
public class RedFar extends SyborgsAutonBase {
	@Override
	public void onStartButtonPressed() {
		Shooter.INSTANCE.setTargetVelocity(2000);
		Paths paths = new Paths(follower());
		new SequentialGroup(
				new FollowPath(paths.Preload),
				Shooter.INSTANCE.shoot(),
				new FollowPath(paths.GPP),
				new FollowPath(paths.GPPReturn),
				Shooter.INSTANCE.shoot(),
				new FollowPath(paths.PGP),
				new FollowPath(paths.PGPReturn),
				Shooter.INSTANCE.shoot(),
				new FollowPath(paths.PPG),
				new FollowPath(paths.PPGReturn),
				Shooter.INSTANCE.shoot(),
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
									new Pose(-17.897, 19.141)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
					.build();

			GPP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-17.897, 19.141),
									new Pose(5.045, 18.257),
									new Pose(24.602, 21.382),
									new Pose(42.319, 19.985),
									new Pose(38.935, 62.575)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			GPPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(38.935, 62.575),
									new Pose(36.418, 15.757),
									new Pose(-17.728, 18.900)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			PGP = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-17.728, 18.900),
									new Pose(13.843, 12.264),
									new Pose(15.505, 30.730),
									new Pose(14.216, 60.400)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			PGPReturn = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(14.216, 60.400),
									new Pose(10.290, 24.486),
									new Pose(-17.773, 18.929)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			PPG = follower.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(-17.773, 18.929),
									new Pose(-8.977, 25.032),
									new Pose(-10.944, 53.437)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();

			PPGReturn = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-10.944, 53.437),
									new Pose(-17.408, 18.937)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
					.build();

			LeaveZone = follower.pathBuilder()
					.addPath(
							new BezierLine(
									new Pose(-17.408, 18.937),
									new Pose(1.961, 51.032)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(90))
					.build();
		}
	}
}
