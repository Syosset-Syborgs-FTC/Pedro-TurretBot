package org.firstinspires.ftc.teamcode.subsytems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.PIDFController;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {

	public volatile static double kP = 0.0002;
	public volatile static double kI = 0.00062;
	public volatile static double kD = 0.00001;
	public volatile static double kF = 0.0062;
	public volatile static double VELOCITY_FILTER_ALPHA = 0.9;
	public static Shooter INSTANCE = new Shooter();

	MotorEx intake = new MotorEx("in");
	MotorEx flywheel = new MotorEx("st");
	PIDFController controller = new PIDFController(kP, kI, kD);
	ServoEx gate = new ServoEx("ga");
	ServoEx rgbLight = new ServoEx("rgb");

	double targetVelocity = 0;
	boolean autoAlignIndicator = false;
	int intakeState = 0;
	boolean shooting = false;

	private double prevEncoderTicks = 0;
	private long prevTime = 0;
	private double currentVelocity = 0;

	public void setShooting(boolean shooting) {
		this.shooting = shooting;
	}

	public void initialize() {
		voltage = ActiveOpMode.hardwareMap().voltageSensor.get("Expansion Hub 2");
		shooting = false;
		intakeState = 0;
		targetVelocity = 0;
		flywheel.zero();
		flywheel.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		prevEncoderTicks = flywheel.getCurrentPosition();
		prevTime = System.nanoTime();
		currentVelocity = 0;

		// override write cache
		intake.setPower(0.02);
		intake.setPower(0);
		gate.setPosition(0.82);
		gate.setPosition(0.8);
		rgbLight.setPosition(0.02);
		rgbLight.setPosition(0);
		flywheel.setPower(0.02);
		flywheel.setPower(0);
		controller.reset();
	}

	public void setIntakeState(int state) {
		this.intakeState = state;
	}

	public void toggleIntake() {
		setIntakeState(this.intakeState == 1 ? 0 : 1);
	}

	public void toggleOuttake() {
		setIntakeState(this.intakeState == -1 ? 0 : -1);
	}

	VoltageSensor voltage;
	private double cachedVoltage = 12.0;
	double loopCount = 0;

	public void setTargetVelocity(double velocity) {
		this.targetVelocity = velocity;
	}

	private void updateRGB() {
		if (targetVelocity == 0) {
			rgbLight.setPosition(0.28); // red
		} else if (Math.abs(targetVelocity - currentVelocity) > 100) {
			rgbLight.setPosition(0.28);
		} else if (currentVelocity < targetVelocity - 20) {
			rgbLight.setPosition(0.611); // blue
		} else if (currentVelocity > targetVelocity + 20) {
			rgbLight.setPosition(0.333); // orange
		} else {
			if (autoAlignIndicator) {
				rgbLight.setPosition(0.999); // white
			} else {
				rgbLight.setPosition(0.5); // green
			}
		}
	}

	@Override
	public void periodic() {
		updateFlywheel();
		updateRGB();
		updateIntakeTransfer();
	}

	public void updateIntakeTransfer() {
		if (!ActiveOpMode.opModeInInit()) {
			if (shooting) {
				gate.setPosition(0.65);
				intake.setPower(1);
			} else {
				gate.setPosition(0.8);
				intake.setPower(intakeState);
			}
		}

		Telemetry telemetry = ActiveOpMode.telemetry();
		telemetry.addData("Currently Shooting", shooting);
		telemetry.addData("Intake State", intakeState);
	}

	private void updateFlywheel() {
		if (loopCount++ % 20 == 0) {
			cachedVoltage = voltage.getVoltage();
		}
		double currentTicks = flywheel.getCurrentPosition();
		long currentTime = System.nanoTime();
		double dt = (currentTime - prevTime) / 1.0E9;
		ActiveOpMode.telemetry().addData("dt", dt);

		if (dt > 0) {
			double rawVelocity = (currentTicks - prevEncoderTicks) / dt;

			currentVelocity = (VELOCITY_FILTER_ALPHA * rawVelocity)
					+ ((1.0 - VELOCITY_FILTER_ALPHA) * currentVelocity);
		}
		prevEncoderTicks = currentTicks;
		prevTime = currentTime;

		controller.setConstants(kP, kI, kD);
		controller.setTarget(targetVelocity);

		double power = controller.update(currentVelocity, kF * targetVelocity / cachedVoltage);
		if (ActiveOpMode.opModeInInit()) return;
		flywheel.setPower(power);

		Telemetry telemetry = ActiveOpMode.telemetry();
		if (flywheel.getMotor().isOverCurrent()) {
			telemetry.addLine("Flywheel is over current!");
		}
		telemetry.addData("Flywheel Error", targetVelocity - currentVelocity);
		telemetry.addData("Flywheel Velocity", currentVelocity);
		telemetry.addData("Flywheel Target", targetVelocity);
		telemetry.addData("Flywheel Power", power);
	}
}
