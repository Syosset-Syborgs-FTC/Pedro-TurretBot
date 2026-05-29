package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.field.Style;
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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueCloseShootLeave extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(-52.070, -45.390, Math.toRadians(229.2)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
		TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(0));

		PathChain preload = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(-52.070, -45.390),
								new Pose(-40.528, -30.125)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(230))
				.build();
		PathChain leave = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(-52.070, -45.390),
								new Pose(-50.528, -14.125)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(230))
				.build();

		Drawing.drawPath(preload, new Style("#FF0000", "#FF0000", 0.75));
		Drawing.sendPacket();

		new ParallelDeadlineGroup(
				new SequentialGroup(
						new Delay(3),
						new FollowPath(preload, false),
						Shooter.INSTANCE.shootCommand(),
						new FollowPath(leave)
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