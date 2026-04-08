package org.firstinspires.ftc.teamcode.subsytems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.control.PIDFController;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {

	public volatile static double kP = 0.01/2;
	public volatile static double kI = 0.00062/2;
	public volatile static double kD = 0;
	public volatile static double kF = 0.0062/2;
	public static Shooter INSTANCE = new Shooter();

	MotorEx intake = new MotorEx("in");
	MotorEx flywheel = new MotorEx("st");
	MotorEx flywheel2 = new MotorEx("st2").reversed();
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
		autoAlignIndicator = false;
		intakeState = 0;
		targetVelocity = 0;
		currentVelocity = 0;
		loopCount = 0;
		cachedVoltage = 12.0;

		flywheel.zero();
		flywheel2.zero();
		flywheel.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		flywheel2.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		prevEncoderTicks = flywheel.getCurrentPosition();
		prevTime = System.nanoTime();
		currentVelocity = 0;
		flywheel.getMotor().setCurrentAlert(7, CurrentUnit.AMPS);


		// override write cache
		Common.overrideCacheZero(rgbLight, gate);
		Common.overrideCacheZero(flywheel, intake);
		controller.reset();
	}
	public void startIntake() {
		setIntakeState(1);
	}
	public void stopIntake() {
		 setIntakeState(0);
	}
	public double getCurrentVelocity() {
		return currentVelocity;
	}

	public double getTargetVelocity() {
		return targetVelocity;
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
		if (!shooting && intakeState == 1) {
			rgbLight.setPosition(0.3); // orange
			return;
		}
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

	public Command shootCommand() {
		return new ParallelDeadlineGroup(
				new Delay(3),
				new InstantCommand(() -> setShooting(true))
		).then(
				new InstantCommand(() -> setShooting(false))
		);
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
		if (loopCount++ % 20 == 0 && voltage != null) {
			cachedVoltage = voltage.getVoltage();
		}
		double currentTicks = flywheel.getCurrentPosition();
		long currentTime = System.nanoTime();
		double dt = (currentTime - prevTime) / 1.0E9;
		ActiveOpMode.telemetry().addData("dt", dt);

		currentVelocity = flywheel.getVelocity();
		prevEncoderTicks = currentTicks;
		prevTime = currentTime;

		controller.setConstants(kP, kI, kD);
		controller.setTarget(targetVelocity);

		double power = controller.update(currentVelocity, kF * targetVelocity / cachedVoltage);
		if (ActiveOpMode.opModeInInit()) return;
		flywheel.setPower(power);
		flywheel2.setPower(power*8/9);
//		flywheel.setPower(1);
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
