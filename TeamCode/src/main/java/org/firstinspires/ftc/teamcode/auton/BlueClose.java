package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.field.Style;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.control.ShooterInterpolator;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueClose extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(-52.070, -45.390, Math.toRadians(229.2)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
		TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(50));
		PathChain preload = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(-52.070, -45.390),
								new Pose(-10.528, -22.125)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(270))
				.build();
		PathChain initialPGP = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(-10.528, -22.125),
								new Pose(22.755, -19.709),
								new Pose(24.099, -59.145)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();
		PathChain initialReturn = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(24.099, -61.145),
								new Pose(13.652, -35.193),
								new Pose(-1.306, -17.489)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();
		PathChain secondClear = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(-1.306, -17.489),
								new Pose(19.475, -36.120),
								new Pose(19, -57.5)
						)
				)
				.setNoDeceleration()
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(235))
				.build();
		PathChain secondMove = follower().pathBuilder().addPath(
						new BezierLine(
								new Pose(19, -57.5),
								new Pose(19.5, -57)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(235))
				.setVelocityConstraint(10)
				.addPath(
						new BezierLine(
								new Pose(19.5, -57),
								new Pose(19, -57.5)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(235))
				.setTimeoutConstraint(100)
				.setVelocityConstraint(15)
				.setNoDeceleration()
				.build();

		PathChain secondReturn = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(19.420, -57.5),
								new Pose(8.967, -35.121),
								new Pose(-1.306, -17.489)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(235), Math.toRadians(270))
				.setVelocityConstraint(0.1)
				.setBrakingStart(1.3)
				.setGlobalDeceleration()
				.build();
		PathChain thirdPPG = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(-1.306, -17.489),
								new Pose(-9, -20),
								new Pose(-9, -53)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
				.setTValueConstraint(0.7)
				.build();
		PathChain thirdReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(-9, -53),
								new Pose(-1.306, -17.489)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
				.build();

		Drawing.drawPath(preload, new Style("#00FFFF", "#00FFFF", 0.75));
		Drawing.sendPacket();
		new ParallelDeadlineGroup(
				new SequentialGroup(
						new FollowPath(preload, false),
						Shooter.INSTANCE.shootCommand(),
						new InstantCommand(Shooter.INSTANCE::startIntake),
						new FollowPath(initialPGP),
//						new Delay(0.2),
						new FollowPath(initialReturn),
						Shooter.INSTANCE.shootCommand(),
						new FollowPath(secondClear),
						new Delay(0.5),
						new FollowPath(secondMove),
						new Delay(0.5),
						new InstantCommand(() -> telemetry.addLine("return done")),
						new FollowPath(secondReturn, false),
						Shooter.INSTANCE.shootCommand(),
						new FollowPath(secondClear),
						new Delay(0.5),
						new FollowPath(secondMove),
						new Delay(0.5),
						new FollowPath(secondReturn, false),
						Shooter.INSTANCE.shootCommand(),
						new FollowPath(thirdPPG),
//						new Delay(0.2),
						new FollowPath(thirdReturn),
						Shooter.INSTANCE.shootCommand(),
						new ParallelGroup(
								new FollowPath(thirdPPG),
								new InstantCommand(() -> TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(0)))
						)
				),
				new LambdaCommand().setUpdate(() -> {
					Pose currentPose = follower().getPose();
					if (Common.alliance == Common.Alliance.Blue) {
						currentPose = new Pose(currentPose.getX(), -currentPose.getY());
					}
					ShooterInterpolator.ShooterState state = ShooterInterpolator.INSTANCE.getTargetState(currentPose);
					TurretAngleControl.INSTANCE.setAnglerPosition(state.hoodAngle);
					Shooter.INSTANCE.setTargetVelocity(state.velocity);
				}).setIsDone(() -> false)
		).schedule();
	}
}
