package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.impl.ServoEx;
import com.photon.photoncore.PhotonCore;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

//@Disabled
@TeleOp
@Configurable
public class TestTeleop extends NextFTCOpMode {
	double targetVelocity = 1350;
	boolean flywheelEnabled = false;
	ServoEx angler = new ServoEx("an");
	AnalogInput potentiometer;
	public TestTeleop() {
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
	}
	int intakeState = 0;
	boolean shootOverride = false;

	@Override
	public void onStartButtonPressed() {
		follower().startTeleopDrive(true);
		gamepad1().options().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.followingAprilTag = !TurretAngleControl.INSTANCE.followingAprilTag);

		gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE::toggleIntake);
		gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE::toggleOuttake);

		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenBecomesTrue(() -> Shooter.INSTANCE.setShooting(true)).whenBecomesFalse(() -> Shooter.INSTANCE.setShooting(false));

		gamepad1().dpadLeft().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.setTurretAngle(TurretAngleControl.INSTANCE.turretTargetAngle - Math.toRadians(90)));
		gamepad1().dpadRight().whenBecomesTrue(() -> TurretAngleControl.INSTANCE.setTurretAngle(TurretAngleControl.INSTANCE.turretTargetAngle + Math.toRadians(90)));

		gamepad1().b().whenBecomesTrue(() -> driveSpeedMultiplier = driveSpeedMultiplier == 1 ? 0.5 : 1);

		gamepad1().y().whenTrue(() -> angler.setPosition(angler.getPosition() + 0.05));
		gamepad1().a().whenTrue(() -> angler.setPosition(angler.getPosition() - 0.05));
		gamepad1().dpadUp().whenTrue(() -> targetVelocity += 25);
		gamepad1().dpadDown().whenTrue(() -> targetVelocity -= 25);
	}
	double driveSpeedMultiplier = 1;
	@Override
	public void onUpdate() {
		follower().setTeleOpDrive( -gamepad1.left_stick_y * driveSpeedMultiplier, -gamepad1.left_stick_x * driveSpeedMultiplier, -gamepad1.right_stick_x * driveSpeedMultiplier, true);
		Shooter.INSTANCE.setTargetVelocity(flywheelEnabled? targetVelocity : 0);

		telemetry.addData("Shooting", shootOverride);
		telemetry.addData("Intake State", intakeState);
		telemetry.addData("Target Velocity", targetVelocity);
		telemetry.addData("Potentiometer", potentiometer.getVoltage());
	}
	public void onStop() {
		LimeLightAprilTag.INSTANCE.stop();
	}
}
