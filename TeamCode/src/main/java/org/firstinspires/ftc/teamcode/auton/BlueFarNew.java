package org.firstinspires.ftc.teamcode.auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class BlueFarNew extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(64.000, -12.000, Math.toRadians(180)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Blue;
		Shooter.INSTANCE.setTargetVelocity(1810);

		TurretAngleControl.INSTANCE.setAnglerPosition(0.7);
		TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(-33));

		PathChain initialGPP = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(61.238, -22.498),
								new Pose(36.559, -24.742),
								new Pose(36.287, -38.105)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
				.build();
		PathChain initialGPPIntake = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(36.287, -38.105),
								new Pose(36.287, -56.105)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(270))
				.build();

		PathChain initialReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(36.287, -56.105),
								new Pose(61.469, -22.895)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
				.build();
		PathChain secondAlign = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(61.469, -22.895),
								new Pose(41.872, -37.759),
								new Pose(42.542, -65.559)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(360))
				.build();
		PathChain secondIntake = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(42.542, -65.559),
								new Pose(57.172, -65.595)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(360))
				.build();
		PathChain secondMove = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(57.172, -65.595),
								new Pose(64.689, -59.652)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(360))
				.build();
		PathChain secondReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(64.689, -59.652),
								new Pose(61.182, -22.835)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(270))
				.build();
		PathChain thirdScoop = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(61.182, -22.835),
								new Pose(64.448, -62.875),
								new Pose(54.168, -65.962),
								new Pose(29.924, -65.013)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
				.build();
		PathChain thirdReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(29.924, -65.013),
								new Pose(61.397, -22.471)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		PathChain leaveZone = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(61.397, -22.471),
								new Pose(60.000, -12.000)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		new SequentialGroup(
				new Delay(3),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(initialGPP),
				new FollowPath(initialGPPIntake),
				new Delay(0.2),
				new FollowPath(initialReturn),
				Shooter.INSTANCE.shootCommand(),
				new InstantCommand(() -> TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(70))),
				new FollowPath(secondAlign),
				new FollowPath(secondIntake, true, 0.4),
				new FollowPath(secondMove),
				new FollowPath(secondReturn),
				Shooter.INSTANCE.shootCommand(),
				new InstantCommand(() -> TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(-30))),
				new FollowPath(thirdScoop),
				new FollowPath(thirdReturn),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(thirdScoop),
				new FollowPath(thirdReturn),
				Shooter.INSTANCE.shootCommand(),
				new FollowPath(leaveZone)

		).schedule();
	}
}
