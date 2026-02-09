package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

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
	MotorEx intake = new MotorEx("in");
	MotorEx turret = new MotorEx("tu");
	ServoEx gate = new ServoEx("ga");
	ServoEx angler = new ServoEx("an");
	AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "pot");

	@Override
	public void onInit() {
		flywheelControl.setGoal(new KineticState(0, 0));
		addComponents(
				TelemetryComponent.INSTANCE,
				new LoopTimeComponent(),
				new PedroComponent(Constants::createFollower),
				BindingsComponent.INSTANCE,
				BulkReadComponent.INSTANCE
		);
	}

	@Override
	public void onStartButtonPressed() {
		follower().startTeleopDrive(true);
	}
	double intakePower = 0.0;
	double mult = 1;
	@Override
	public void onUpdate() {
		follower().setTeleOpDrive( -gamepad1.left_stick_y * mult, -gamepad1.left_stick_x * mult, -gamepad1.right_stick_x * mult, false);
		gamepad1().rightBumper().whenBecomesTrue(() -> intakePower = intakePower == 1.0 ? 0.0 : 1.0);
		gamepad1().leftBumper().whenBecomesTrue(() -> intakePower = intakePower == -1.0 ? 0.0 : -1.0);
		gamepad1().leftTrigger().atLeast(0.5).whenBecomesTrue(() -> flywheelEnabled = !flywheelEnabled);
		gamepad1().rightTrigger().atLeast(0.5).toggleOnBecomesTrue().whenBecomesTrue(() -> {
			gate.setPosition(1);
			intakePower = 1;
		}).whenBecomesFalse(() -> {
			gate.setPosition(0);
			intakePower = 0;
		});
		flywheelControl.setGoal(new KineticState(0, flywheelEnabled? 0 : targetVelocity));
		flywheel.setPower(flywheelControl.calculate(flywheel.getState()));
		intake.setPower(intakePower);
		gamepad1().dpadLeft().whenTrue(() -> turretAngle -= Math.toRadians(10));
		gamepad1().dpadRight().whenTrue(() -> turretAngle += Math.toRadians(10));
		gamepad1().y().whenTrue(new SetPosition(angler, angler.getPosition() + 0.05));
		gamepad1().a().whenTrue(new SetPosition(angler, angler.getPosition() - 0.05));
		gamepad1().dpadUp().whenTrue(() -> targetVelocity += 25);
		gamepad1().dpadDown().whenTrue(() -> targetVelocity -= 25);
		turret.setPower(turretControl.calculate(turret.getState()));

		telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
		telemetry.addData("Turret Angle", turret.getCurrentPosition());
		telemetry.addData("Potentiometer", potentiometer.getVoltage());
	}
}
