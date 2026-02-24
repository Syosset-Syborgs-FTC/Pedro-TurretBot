package org.firstinspires.ftc.teamcode.auton;

import com.photon.photoncore.PhotonCore;

import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

public class SyborgsAutonBase extends NextFTCOpMode {

	@Override
	public void onInit() {
		PhotonCore.enable();
		Shooter.INSTANCE.setTargetVelocity(0);
	}
	public SyborgsAutonBase() {
		addComponents(
				TelemetryComponent.INSTANCE,
				new LoopTimeComponent(),
				new PedroComponent(Constants::createFollower),
				BindingsComponent.INSTANCE,
				PanelsPacketComponent.INSTANCE,
				BulkReadComponent.INSTANCE,
				new SubsystemComponent(Shooter.INSTANCE, TurretAngleControl.INSTANCE)
		);
	}
	public void onStop() {
		LimeLightAprilTag.INSTANCE.stop();
	}
}
