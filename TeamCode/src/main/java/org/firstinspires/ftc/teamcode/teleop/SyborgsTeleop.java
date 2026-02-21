package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.gamepad1;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.photon.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

//@Disabled
@TeleOp
@Configurable
public class SyborgsTeleop extends NextFTCOpMode {
	double targetVelocity = 2050;
	boolean flywheelEnabled = false;
	AnalogInput potentiometer;
	public SyborgsTeleop() {
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

	@Override
	public void onInit() {
		PhotonCore.enable();
		Shooter.INSTANCE.setTargetVelocity(0);
		potentiometer = hardwareMap.get(AnalogInput.class, "pot");

		gamepad1().rightBumper()
				.whenBecomesTrue(() -> Common.alliance = Common.alliance.getOpposite());
	}
	public void onWaitForStart() {
		telemetry.addData("Alliance (press right bumper to change)", Common.alliance);
	}
	double headingOffset = 0;

	@Override
	public void onStartButtonPressed() {
		gamepad1().rightBumper().clear$bindings();
		follower().startTeleopDrive(true);
		gamepad1().x().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.followingAprilTag = !TurretAngleControl.INSTANCE.followingAprilTag);
		gamepad1().options().whenBecomesTrue(() -> headingOffset = SensorFusion.INSTANCE.getRawPinpointHeading());
		gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE::toggleIntake);
		gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE::toggleOuttake);

		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenBecomesTrue(() -> Shooter.INSTANCE.setShooting(true)).whenBecomesFalse(() -> Shooter.INSTANCE.setShooting(false));

		gamepad1().dpadLeft().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.setTurretAngle(TurretAngleControl.INSTANCE.turretTargetAngle - Math.toRadians(15)));
		gamepad1().dpadRight().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.setTurretAngle(TurretAngleControl.INSTANCE.turretTargetAngle + Math.toRadians(15)));

		gamepad1().b().whenBecomesTrue(() -> driveSpeedMultiplier = driveSpeedMultiplier == 1 ? 0.5 : 1);

		gamepad1().y().whenTrue(() -> TurretAngleControl.INSTANCE.offsetAnglerPosition(0.05));
		gamepad1().a().whenTrue(() -> TurretAngleControl.INSTANCE.offsetAnglerPosition(-0.05));
		gamepad1().dpadUp().whenTrue(() -> targetVelocity += 25);
		gamepad1().dpadDown().whenTrue(() -> targetVelocity -= 25);
	}
	double driveSpeedMultiplier = 1;
	Vector drive = new Vector();
	@Override
	public void onUpdate() {
		drive.setOrthogonalComponents( -gamepad1.left_stick_y * driveSpeedMultiplier, -gamepad1.left_stick_x * driveSpeedMultiplier);
		drive.rotateVector(-SensorFusion.INSTANCE.getRawPinpointHeading() + headingOffset);
		follower().setTeleOpDrive(drive.getXComponent(), drive.getYComponent(), -gamepad1.right_stick_x * driveSpeedMultiplier, true);
		Shooter.INSTANCE.setTargetVelocity(flywheelEnabled? targetVelocity : 0);
	}
	public void onStop() {
		LimeLightAprilTag.INSTANCE.stop();
	}
}
