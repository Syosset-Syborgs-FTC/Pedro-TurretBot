package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp
public class TestTeleop extends NextFTCOpMode {
	ControlSystem turretControl = ControlSystem.builder()
			.posPid(0.01, 0.0001, 0.001)
			.build();
	double targetVelocity = 1350;
	boolean flywheelEnabled = false;
	double turretAngle = 0;
	ServoEx angler = new ServoEx("an");
	AnalogInput potentiometer;
	MotorEx turret = new MotorEx("tu");
	public TestTeleop() {
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
		turretControl.setGoal(new KineticState());
		Shooter.INSTANCE.setTargetVelocity(0);
		potentiometer = hardwareMap.get(AnalogInput.class, "pot");
	}
	int intakeState = 0;
	boolean shootOverride = false;

	@Override
	public void onStartButtonPressed() {
		turretAngle = 0;
		turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turret.zero();
		follower().startTeleopDrive(true);

		gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE::toggleIntake);
		gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE::toggleOuttake);

		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenBecomesTrue(() -> Shooter.INSTANCE.setShooting(true)).whenBecomesFalse(() -> Shooter.INSTANCE.setShooting(false));

		gamepad1().dpadLeft().whenBecomesTrue(() -> turretAngle -= Math.toRadians(10));
		gamepad1().dpadRight().whenBecomesTrue(() -> turretAngle += Math.toRadians(10));

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
		turretControl.setGoal(new KineticState(turretAngle/(2*Math.PI)*5*28));
		turret.setPower(turretControl.calculate(turret.getState()));
		telemetry.addData("Shooting", shootOverride);
		telemetry.addData("Intake State", intakeState);
		telemetry.addData("Target Velocity", targetVelocity);
		telemetry.addData("Target Angle", turretAngle/(2*Math.PI)*5);
		telemetry.addData("Turret Angle", turret.getCurrentPosition());
		telemetry.addData("Potentiometer", potentiometer.getVoltage());
	}
}
