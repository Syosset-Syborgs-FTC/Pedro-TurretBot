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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueCloseNew extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(-52.070, -45.390, Math.toRadians(229.2)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
//		TurretAngleControl.INSTANCE.setTurretAngle(140);
		TurretAngleControl.INSTANCE.followingAprilTag = true;
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
								new Pose(19.755, -19.709),
								new Pose(15.099, -51.145)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();
		PathChain initialReturn = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(15.099, -51.145),
								new Pose(13.652, -35.193),
								new Pose(-1.306, -14.489)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();
		PathChain secondClear = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(-1.306, -14.489),
								new Pose(11.475, -36.120),
								new Pose(19.420, -62.726)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(210))
				.build();
		PathChain secondReturn = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(19.420, -62.726),
								new Pose(8.967, -35.121),
								new Pose(1.879, -14.426)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(270))
				.build();

		Drawing.drawPath(preload, new Style("#00FFFF", "#00FFFF", 0.75));
		Drawing.sendPacket();
		new ParallelDeadlineGroup(
				new SequentialGroup(
						new FollowPath(preload),
						 Shooter.INSTANCE.shootCommand(),
						new InstantCommand(Shooter.INSTANCE::startIntake),
						new FollowPath(initialPGP),
						new Delay(1.5),
						new FollowPath(initialReturn),
						Shooter.INSTANCE.shootCommand(),
						new FollowPath(secondClear),
						new Delay(2),
						new FollowPath(secondReturn),
						Shooter.INSTANCE.shootCommand()
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
