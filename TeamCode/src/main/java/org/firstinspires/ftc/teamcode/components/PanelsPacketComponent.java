package org.firstinspires.ftc.teamcode.components;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.localizer.PoseFilter;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;

import dev.nextftc.core.components.Component;

public class PanelsPacketComponent implements Component {
	public static final PanelsPacketComponent INSTANCE = new PanelsPacketComponent();
	@Override
	public void postUpdate() {
		Drawing.drawPoseHistory(follower().getPoseHistory());
		Drawing.drawRobot(follower().getPose());
		if (follower().getCurrentPathChain() != null) {
			Drawing.drawPath(follower().getCurrentPathChain(), new Style("#00FFFF", "#00FFFF", 0.75));
		}
		SensorFusion.INSTANCE.cachedMT1Pose.ifPresent(p -> Drawing.drawRobot(p, "#4CAF50"));
		SensorFusion.INSTANCE.cachedMT2Pose.ifPresent(p -> Drawing.drawRobot(p, "#FF5722"));
		SensorFusion.INSTANCE.cachedCameraPose.ifPresent(p -> Drawing.drawRobot(p, "#FFFFFF"));

		Drawing.sendPacket();
	}
	@Override
	public void postWaitForStart() {
		Drawing.sendPacket();
	}
}
