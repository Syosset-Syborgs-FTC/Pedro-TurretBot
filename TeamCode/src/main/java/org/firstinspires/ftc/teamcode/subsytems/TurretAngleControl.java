package org.firstinspires.ftc.teamcode.subsytems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.control.TrapezoidalInterpolator;
import org.firstinspires.ftc.teamcode.control.TurretVelocityCompensator;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.Positionable;

@Configurable
public class TurretAngleControl implements Subsystem {
	public static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0.0000000001, 0.05);//old vals (1, 0.0000000001, 0.03);
	// profile constraints
	public static double MAX_VEL = (1150.0 / 4.0 / 60.0) * 20 * Math.PI;//switched this with new  motor
	public static double MAX_ACCEL = MAX_VEL * .04; // reach max speed in 0.4s

	public static final TurretAngleControl INSTANCE = new TurretAngleControl();

	AnalogInput potentiometer;
	public MotorEx turret = new MotorEx("tu", 0.001).reversed();
	ServoEx angler = new ServoEx("an");

	public boolean followingAprilTag = false;
	public double turretTargetAngle = 0;
	public double scalar = 1.0 / 145.1 * Math.PI * (30.0 / 120.0) * 2.0;//switched this in code as well

	ControlSystem turretControl = ControlSystem.builder()
			.posPid(pidCoefficients)
//			.interpolator(new TrapezoidalInterpolator(MAX_VEL, MAX_ACCEL))
			.feedforward(TurretVelocityCompensator.INSTANCE)
			.build();

	@Override
	public void initialize() {
		followingAprilTag = false;
		turretTargetAngle = 0;
		angler.getServo().setDirection(Servo.Direction.REVERSE);
		turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turret.zero();
		turretControl.reset();
		turretControl.setGoal(new KineticState());
		Common.overrideCacheZero(angler);
		Common.overrideCacheZero(turret);
		potentiometer = ActiveOpMode.hardwareMap().get(AnalogInput.class, "pot");
		LimeLightAprilTag.INSTANCE.initialize();
	}

	@Override
	public void periodic() {
		Telemetry telemetry = ActiveOpMode.telemetry();
		telemetry.addData("Angler Position", angler.getPosition());
//		telemetry.addData("pot voltage", potentiometer.getVoltage());

		turretControl.setGoal(new KineticState(turretTargetAngle));
		double turretPower = turretControl.calculate(turret.getState().times(scalar));

		double actualTurretAngle = getActualTurretAngle();
		LimeLightAprilTag.INSTANCE.setTurretAngle(actualTurretAngle);
		LimeLightAprilTag.INSTANCE.periodic();
		if (ActiveOpMode.opModeInInit()) return;
		turret.setPower(turretPower);

		Vector alignTarget = new Vector();
		if (Common.alliance == Common.Alliance.Red) {
			alignTarget.setOrthogonalComponents(-62.5, 69);
		} else {
			alignTarget.setOrthogonalComponents(-58, -69);
		}

		if (followingAprilTag) {
			double robotHeading = SensorFusion.INSTANCE.getPose().getHeading();
			setTurretAngle(robotHeading - alignTarget.minus(SensorFusion.INSTANCE.getPose().getAsVector()).getTheta());
		}
		telemetry.addData("Turret Target Angle", Math.toDegrees(turretTargetAngle));
		telemetry.addData("Turret Profile Position", Math.toDegrees(turretControl.getReference().getPosition()));
		telemetry.addData("Turret Actual Angle", Math.toDegrees(actualTurretAngle));
		telemetry.addData("Turret Power", turretPower);
	}

	public double getActualTurretAngle() {
		return turret.getCurrentPosition() * scalar;
	}

	public void setAnglerPosition(double pos) { angler.setPosition(Range.clip(pos, 0, 0.8)); }// from .85 to .8
	public void offsetAnglerPosition(double pos) { setAnglerPosition(getAnglerPosition() + pos); }
	public double getAnglerPosition() { return angler.getPosition(); }
	public double getTurretAngle() { return turretTargetAngle; }

	public void setTurretAngle(double angle) {
		angle = wrapAngle(angle);
		this.turretTargetAngle = angle;
	}
	public void offsetTurretAngle(double offset) {
		setTurretAngle(getTurretAngle() + offset);
	}
	public static double wrapAngle(double angleRadians) {
		double min = Math.toRadians(-215);

		double shifted = angleRadians - min;
		double result = shifted % (2 * Math.PI);

		if (result < 0) {
			result += 2 * Math.PI;
		}

		return result + min;
	}
}
