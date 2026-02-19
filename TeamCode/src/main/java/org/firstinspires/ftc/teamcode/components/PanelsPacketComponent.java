package org.firstinspires.ftc.teamcode.components;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.field.Style;


import org.firstinspires.ftc.teamcode.Drawing;

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
		Drawing.sendPacket();
	}
	@Override
	public void postWaitForStart() {
		Drawing.sendPacket();
	}
}
