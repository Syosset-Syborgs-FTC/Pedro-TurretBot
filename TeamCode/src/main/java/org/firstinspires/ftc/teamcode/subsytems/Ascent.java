package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.GravityFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.hardware.impl.MotorEx;

public class Ascent implements Subsystem {
	public static final Ascent INSTANCE = new Ascent();
	// todo: tune
	PIDCoefficients pidCoefficients = new PIDCoefficients(0.1, 0, 0);
	GravityFeedforwardParameters ffParams = new GravityFeedforwardParameters(0.1, 0);
	MotorEx ascentMotor = new MotorEx("asc");
	ControlSystem pid = ControlSystem.builder().posPid(pidCoefficients)
			.armFF(ffParams)
			.build();
	Angle targetAngle = Angle.fromDeg(0);
	double oneRev = 5281.1;
	@Override
	public void initialize() {
		pid.reset();
		ascentMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		ascentMotor.zero();
		setAngle(Angle.fromDeg(0));
	}
	public void setAngle(Angle angle) {
		angle = Angle.fromDeg(Math.max(angle.inDeg, 90));
		targetAngle = angle;
		pid.setGoal(new KineticState(angle.inRad, 0, 0));
	}
	public Angle getAngle() {
		return targetAngle;
	}
	@Override
	public void periodic() {
		ascentMotor.setPower(pid.calculate(ascentMotor.getState().times(2*Math.PI/oneRev)));
	}
}