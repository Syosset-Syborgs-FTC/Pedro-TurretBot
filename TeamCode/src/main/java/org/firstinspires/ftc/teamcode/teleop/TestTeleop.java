package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp
public class TestTeleop extends NextFTCOpMode {
	ControlSystem flywheelControl = ControlSystem.builder()
			.velPid(0.002, 0.00052, 0.0)
			.basicFF(0.0052, 0.08, 0.0)
			.build();
	ControlSystem turretControl = ControlSystem.builder()
			.posPid(0.01, 0.0001, 0.001)
			.build();
	double targetVelocity = 1350;
	boolean flywheelEnabled = false;
	double turretAngle = 0;
	MotorEx flywheel = new MotorEx("st");
	MotorEx intake = new MotorEx("in").reversed();
	MotorEx turret = new MotorEx("tu");
	ServoEx gate = new ServoEx("ga");
	ServoEx angler = new ServoEx("an");
	AnalogInput potentiometer;
	public TestTeleop() {
		addComponents(
				TelemetryComponent.INSTANCE,
				new LoopTimeComponent(),
				new PedroComponent(Constants::createFollower),
				BindingsComponent.INSTANCE,
				BulkReadComponent.INSTANCE
		);
	}

	@Override
	public void onInit() {
		flywheelControl.setGoal(new KineticState(0, 0));
		potentiometer = hardwareMap.get(AnalogInput.class, "pot");
	}
	int intakeState = 0;
	boolean shootOverride = false;

	@Override
	public void onStartButtonPressed() {
		follower().startTeleopDrive(true);
		gamepad1().rightBumper().whenBecomesTrue(() -> intakeState = intakeState == 1 ? 0 : 1);
		gamepad1().leftBumper().whenBecomesTrue(() -> intakeState = intakeState == -1 ? 0 : -1);
		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).whenTrue(() -> {
			gate.setPosition(1);
			shootOverride = true;
		}).whenFalse(() -> {
			gate.setPosition(0);
			shootOverride = false;
		});
		gamepad1().dpadLeft().whenTrue(() -> turretAngle -= Math.toRadians(10));
		gamepad1().dpadRight().whenTrue(() -> turretAngle += Math.toRadians(10));
		gamepad1().b().whenBecomesTrue(() -> driveSpeedMultiplier = driveSpeedMultiplier == 1 ? 0.5 : 1);
		gamepad1().y().whenTrue(new SetPosition(angler, angler.getPosition() + 0.05));
		gamepad1().a().whenTrue(new SetPosition(angler, angler.getPosition() - 0.05));
		gamepad1().dpadUp().whenTrue(() -> targetVelocity += 25);
		gamepad1().dpadDown().whenTrue(() -> targetVelocity -= 25);
	}
	double driveSpeedMultiplier = 1;
	@Override
	public void onUpdate() {
		follower().setTeleOpDrive( -gamepad1.left_stick_y * driveSpeedMultiplier, -gamepad1.left_stick_x * driveSpeedMultiplier, -gamepad1.right_stick_x * driveSpeedMultiplier, true);
		flywheelControl.setGoal(new KineticState(0, flywheelEnabled? 0 : targetVelocity));
		flywheel.setPower(flywheelControl.calculate(flywheel.getState()));
		intake.setPower(shootOverride? 1 : intakeState);
		turretControl.setGoal(new KineticState(turretAngle/(2*Math.PI)*5*28));
		turret.setPower(turretControl.calculate(turret.getState()));
		telemetry.addData("Shooting", shootOverride);
		telemetry.addData("Intake State", intakeState);
		telemetry.addData("Target Velocity", targetVelocity);
		telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
		telemetry.addData("Turret Angle", turret.getCurrentPosition());
		telemetry.addData("Potentiometer", potentiometer.getVoltage());
	}
}
