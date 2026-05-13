package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.gamepad1;

import static dev.nextftc.ftc.Gamepads.gamepad2;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.photon.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.RumbleComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.control.ShooterInterpolator.ShooterState;
import org.firstinspires.ftc.teamcode.control.ShooterInterpolator;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsytems.Ascent;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

//@Disabled
@TeleOp(group = "Teleop")
@Configurable
public class SyborgsTeleop extends NextFTCOpMode {
	double targetVelocity = 1550;
	boolean flywheelEnabled = false;
//	AnalogInput potentiometer;
	public SyborgsTeleop() {
		addComponents(
				TelemetryComponent.INSTANCE,
				new LoopTimeComponent(),
				new PedroComponent(Constants::createFollower),
				BindingsComponent.INSTANCE,
				PanelsPacketComponent.INSTANCE,
				BulkReadComponent.INSTANCE,
				RumbleComponent.INSTANCE,
				new SubsystemComponent(Shooter.INSTANCE, TurretAngleControl.INSTANCE
//						, Ascent.INSTANCE
				)
		);
	}
	boolean autoPowerAngle = false;
	@Override
	public void onInit() {
		headingOffset = 0;
		autoPowerAngle = false;
		flywheelEnabled = false;
		PhotonCore.enable();
		Shooter.INSTANCE.setTargetVelocity(0);
//		potentiometer = hardwareMap.get(AnalogInput.class, "pot");
		gamepad1().rightBumper().whenBecomesTrue(() -> Common.alliance = Common.alliance.getOpposite());
	}
	public void onWaitForStart() {
		telemetry.addData("Alliance (press right bumper to change)", Common.alliance);
	}
	double headingOffset = 0;

	@Override
	public void onStartButtonPressed() {
		Shooter.INSTANCE.setTargetVelocity(targetVelocity);
		TurretAngleControl.INSTANCE.setAnglerPosition(0.5);
		gamepad1().rightBumper().clear$bindings();
		follower().startTeleopDrive(true);
		gamepad1().options().whenBecomesTrue(() -> headingOffset = SensorFusion.INSTANCE.getRawPinpointHeading());
		//gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE::toggleIntake);
		gamepad1().rightBumper().whenBecomesTrue(() -> {
			Shooter.INSTANCE.toggleIntake();
			RumbleComponent.INSTANCE.toggle();
		});
		gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE::toggleOuttake);

		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenBecomesTrue(() -> Shooter.INSTANCE.setShooting(true)).whenBecomesFalse(() -> Shooter.INSTANCE.setShooting(false));

		gamepad1().dpadUp().whenTrue(() -> targetVelocity += 10);
		gamepad1().dpadDown().whenTrue(() -> targetVelocity -= 10);
		gamepad1().dpadLeft().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.offsetTurretAngle(-Math.toRadians(15)));
		gamepad1().dpadRight().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.offsetTurretAngle(Math.toRadians(15)));

		gamepad1().y().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.followingAprilTag = !TurretAngleControl.INSTANCE.followingAprilTag);
		gamepad1().a().whenBecomesTrue(() -> driveSpeedMultiplier = driveSpeedMultiplier == 1 ? 0.35 : 1);
		gamepad1().x().whenBecomesTrue(() -> autoPowerAngle = !autoPowerAngle);

		gamepad2().y().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.offsetAnglerPosition(0.05));
		gamepad2().a().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.offsetAnglerPosition(-0.05));
//		gamepad2().dpadUp().whenBecomesTrue(() -> Ascent.INSTANCE.setAngle(Ascent.INSTANCE.getAngle().plus(Angle.fromDeg(15))));
		gamepad2().rightBumper().whenBecomesTrue(() -> Common.alliance = Common.alliance.getOpposite());
	}


	double driveSpeedMultiplier = 1;
	Vector drive = new Vector();
	@Override
	public void onUpdate() {
		drive.setOrthogonalComponents( -gamepad1.left_stick_y * driveSpeedMultiplier, -gamepad1.left_stick_x * driveSpeedMultiplier);
		drive.rotateVector(-SensorFusion.INSTANCE.getRawPinpointHeading() + headingOffset);
		follower().setTeleOpDrive(drive.getXComponent(), drive.getYComponent(), -gamepad1.right_stick_x * driveSpeedMultiplier, true);
		if (autoPowerAngle) {
			Pose currentPose = follower().getPose();
			if (Common.alliance == Common.Alliance.Blue) {
				currentPose = new Pose(currentPose.getX(), -currentPose.getY());
			}
			ShooterState state = ShooterInterpolator.INSTANCE.getTargetState(currentPose);
			TurretAngleControl.INSTANCE.setAnglerPosition(state.hoodAngle);
			Shooter.INSTANCE.setTargetVelocity(flywheelEnabled? state.velocity : 1000); //to stop it to going direct 0
		} else {
			Shooter.INSTANCE.setTargetVelocity(flywheelEnabled? targetVelocity : 0);
		}
		telemetry.addData("pose", follower().getPose());
	}
	public void onStop() {
		LimeLightAprilTag.INSTANCE.stop();
	}
}
