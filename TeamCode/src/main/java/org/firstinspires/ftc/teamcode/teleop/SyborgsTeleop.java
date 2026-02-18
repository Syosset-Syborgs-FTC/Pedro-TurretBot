package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.gamepad1;
import static dev.nextftc.ftc.Gamepads.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;

@TeleOp
public class SyborgsTeleop extends NextFTCOpMode {

	double targetVelocity = 1350;
	boolean flywheelEnabled = false;

	public SyborgsTeleop() {
		addComponents(
				TelemetryComponent.INSTANCE,
				new LoopTimeComponent(),
				new PedroComponent(Constants::createFollower),
				BindingsComponent.INSTANCE,
				BulkReadComponent.INSTANCE,
				new SubsystemComponent(Shooter.INSTANCE)
		);
	}

	@Override
	public void onInit() {
	}


	@Override
	public void onStartButtonPressed() {
		follower().startTeleopDrive(true);

		gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE::toggleIntake);
		gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE::toggleOuttake);

		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenBecomesTrue(() -> Shooter.INSTANCE.setShooting(true)).whenBecomesFalse(() -> Shooter.INSTANCE.setShooting(false));

//		gamepad1().dpadLeft().whenTrue(() -> turretAngle -= Math.toRadians(10));
//		gamepad1().dpadRight().whenTrue(() -> turretAngle += Math.toRadians(10));

		gamepad1().b().whenBecomesTrue(() -> driveSpeedMultiplier = driveSpeedMultiplier == 1 ? 0.5 : 1);

//		gamepad1().y().whenTrue(new SetPosition(angler, angler.getPosition() + 0.05));
//		gamepad1().a().whenTrue(new SetPosition(angler, angler.getPosition() - 0.05));
		gamepad2().dpadUp().whenTrue(() -> targetVelocity += 25);
		gamepad2().dpadDown().whenTrue(() -> targetVelocity -= 25);
	}

	double driveSpeedMultiplier = 1;

	@Override
	public void onUpdate() {
		follower().setTeleOpDrive(-gamepad1.left_stick_y * driveSpeedMultiplier, -gamepad1.left_stick_x * driveSpeedMultiplier, -gamepad1.right_stick_x * driveSpeedMultiplier, true);
		Shooter.INSTANCE.setTargetVelocity(flywheelEnabled ? 0 : targetVelocity);
	}
}
