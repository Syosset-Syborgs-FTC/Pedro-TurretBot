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
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(group = "Auton")
public class RedFar extends SyborgsAutonBase {
	@Override
	public void onInit() {
		super.onInit();
		follower().setPose(new Pose(67.44, 12.69, Math.toRadians(180)));
	}

	@Override
	public void onStartButtonPressed() {
		Common.alliance = Common.Alliance.Red;
		Shooter.INSTANCE.setTargetVelocity(1850);

		TurretAngleControl.INSTANCE.setAnglerPosition(0.7);
		TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(22));

		PathChain initialGPP = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(67.44, 12.69),
								new Pose(41.559, 22.742),
								new Pose(41.287, 24.105)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-270))
				.build();
		PathChain initialGPPIntake = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(41.287, 24.105),
								new Pose(41.287, 63.105)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(-270))
				.build();

		PathChain initialReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(36.287, 63.105),
								new Pose(61.469, 22.895)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(-270))
				.build();
		PathChain secondAlign = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(61.469, 22.895),
								new Pose(41.872, 37.759),
								new Pose(42.542, 65.559)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(360))
				.build();
		PathChain secondIntake = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(42.542, 65.559),
								new Pose(57.172, 65.595)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(360))
				.build();
		PathChain secondMove = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(57.172, 65.595),
								new Pose(64.689, 59.652)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(360))
				.build();
		PathChain secondReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(64.689, 59.652),
								new Pose(61.182, 22.835)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(-270))
				.build();
		PathChain thirdScoop = follower().pathBuilder()
				.addPath(
						new BezierCurve(
								new Pose(61.182, 22.835),
								new Pose(64.448, 62.875),
								new Pose(54.168, 65.962),
								new Pose(29.924, 65.013)
						)
				)
				.setLinearHeadingInterpolation(Math.toRadians(-270), Math.toRadians(180))
				.build();
		PathChain thirdReturn = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(29.924, 65.013),
								new Pose(61.397, 22.471)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		PathChain leaveZone = follower().pathBuilder()
				.addPath(
						new BezierLine(
								new Pose(61.397, 22.471),
								new Pose(60.000, 12.000)
						)
				)
				.setConstantHeadingInterpolation(Math.toRadians(180))
				.build();
		new SequentialGroup(
				new Delay(3),
				Shooter.INSTANCE.shootCommand(),
				new InstantCommand(Shooter.INSTANCE::startIntake),
				new FollowPath(initialGPP),
				new ParallelRaceGroup(
						new FollowPath(initialGPPIntake),
						new Delay(0.8)
				),
				new Delay(0.2),
				new InstantCommand(() -> TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(-73))),
				new ParallelRaceGroup(
						new FollowPath(initialReturn),
						new Delay(2)
				),
				Shooter.INSTANCE.shootCommand(),
				new Delay(1),
//				new InstantCommand(() -> TurretAngleControl.INSTANCE.setTurretAngle(Math.toRadians(-70))),
//				new FollowPath(secondAlign),
//				new FollowPath(secondIntake, true, 0.4),
//				new FollowPath(secondMove),
//				new FollowPath(secondReturn),
//				Shooter.INSTANCE.shootCommand(),
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

