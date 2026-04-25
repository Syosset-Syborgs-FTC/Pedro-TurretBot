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

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
@Autonomous(group = "Auton")
public class ZayansBlueGateAuton extends SyborgsAutonBase {
    @Override
    public void onInit() {
        super.onInit();
        // Updated starting pose to match the first point of your new MainChain
        follower().setPose(new Pose(40.000, 100.000, Math.toRadians(180)));
    }

    @Override
    public void onStartButtonPressed() {
        Common.alliance = Common.Alliance.Blue;
        Shooter.INSTANCE.setTargetVelocity(1670);
        TurretAngleControl.INSTANCE.setAnglerPosition(0.6);

        Paths paths = new Paths(follower());

        new SequentialGroup(
                // Since the new path is one big MainChain, we follow it all at once
                new FollowPath(paths.MainChain),

                // Note: If you need to shoot DURING this path,
                // you will need to add Callbacks to the PathBuilder in the Paths class.
                new ParallelRaceGroup(
                        new WaitUntil(() -> Math.abs(Shooter.INSTANCE.getCurrentVelocity() - Shooter.INSTANCE.getTargetVelocity()) < 15),
                        new Delay(4)
                ),
                Shooter.INSTANCE.shootCommand()
        ).schedule();
    }

    public static class Paths {
        public PathChain MainChain;

        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40, 100), new Pose(45, 58)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(45, 58), new Pose(15, 57.4)))
                    .setTangentHeadingInterpolation()

                    .addPath(new BezierCurve(new Pose(15, 57.4), new Pose(65, 75.023), new Pose(40, 100)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(40, 100), new Pose(45, 65)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(45, 65), new Pose(15, 65)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(15, 65), new Pose(13, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .addPath(new BezierLine(new Pose(13, 60), new Pose(11, 60)))
                    .setConstantHeadingInterpolation(Math.toRadians(145))

                    .addPath(new BezierLine(new Pose(11, 60), new Pose(15, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(15, 60), new Pose(60, 70)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(60, 70), new Pose(13, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))

                    .addPath(new BezierLine(new Pose(13, 60), new Pose(11, 60)))
                    .setConstantHeadingInterpolation(Math.toRadians(145))

                    .addPath(new BezierLine(new Pose(11, 60), new Pose(15, 60)))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(15, 60), new Pose(60, 70)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))

                    .addPath(new BezierLine(new Pose(60, 70), new Pose(50, 70)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}