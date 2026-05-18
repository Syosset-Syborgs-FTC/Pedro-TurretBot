package org.firstinspires.ftc.teamcode.wrapper;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class BezierLineWrapper extends BezierLine {
	private static Pose convert(Pose p) {
		if (p == null) return null;
		Pose pose = InvertedFTCCoordinates.INSTANCE.convertFromPedro(p);
		return new Pose (pose.getX(), pose.getY(), pose.getHeading(), PedroCoordinates.INSTANCE);
	}

	private static FuturePose convert(FuturePose fp) {
		if (fp == null) return null;
		return new FuturePose() {
			@Override
			public boolean initialized() {
				return fp.initialized();
			}

			@Override
			public Pose getPose() {
				return convert(fp.getPose());
			}
		};
	}

	private static ArrayList<Pose> convertPoses(ArrayList<Pose> poses) {
		if (poses == null) return null;
		return poses.stream().map(BezierLineWrapper::convert).collect(Collectors.toCollection(ArrayList::new));
	}


	public BezierLineWrapper(Pose startPose, Pose endPose) {
		super(convert(startPose), convert(endPose));
	}

	public BezierLineWrapper(FuturePose startPose, FuturePose endPose) {
		super(convert(startPose), convert(endPose));
	}

	public BezierLineWrapper(Pose startPose, Pose endPose, boolean initialize) {
		super(convert(startPose), convert(endPose), initialize);
	}
}
