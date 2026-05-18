package org.firstinspires.ftc.teamcode.wrapper;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class BezierCurveWrapper extends BezierCurve {


	private static Pose convert(Pose p) {
		if (p == null) return null;
		Pose pose = InvertedFTCCoordinates.INSTANCE.convertFromPedro(p);
		return new Pose(pose.getX(), pose.getY(), pose.getHeading(), PedroCoordinates.INSTANCE);
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

	private static List<Pose> convertPoses(List<Pose> poses) {
		if (poses == null) return null;
		return poses.stream().map(BezierCurveWrapper::convert).collect(Collectors.toList());
	}
	private static FuturePose[] convertFutures(FuturePose... poses) {
		if (poses == null) return null;
		return Arrays.stream(poses).map(BezierCurveWrapper::convert).toArray(FuturePose[]::new);
	}

	private static List<FuturePose> convertFutureList(List<FuturePose> poses) {
		if (poses == null) return null;
		return poses.stream().map(BezierCurveWrapper::convert).collect(Collectors.toList());
	}

	public BezierCurveWrapper() {
		super();
	}

	public BezierCurveWrapper(List<Pose> controlPoints, PathConstraints constraints) {
		super(convertPoses(controlPoints), constraints);
	}

	protected BezierCurveWrapper(PathConstraints constraints, List<FuturePose> controlPoints) {
		super(constraints, convertFutureList(controlPoints));
	}

	public BezierCurveWrapper(List<Pose> controlPoints) {
		super(convertPoses(controlPoints));
	}

	public BezierCurveWrapper(PathConstraints constraints, FuturePose... controlPoints) {
		super(constraints, convertFutures(controlPoints));
	}

	public BezierCurveWrapper(FuturePose... controlPoints) {
		super(convertFutures(controlPoints));
	}
}