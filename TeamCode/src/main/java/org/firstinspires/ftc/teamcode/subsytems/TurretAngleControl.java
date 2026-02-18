package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.AnalogInput;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

public class TurretAngleControl implements Subsystem {
	public static final TurretAngleControl INSTANCE = new TurretAngleControl();

	AnalogInput potentiometer;
	MotorEx turret = new MotorEx("tu");

	ServoEx angler = new ServoEx("an");

	@Override
	public void initialize() {
		angler.setPosition(0);
		potentiometer = ActiveOpMode.hardwareMap().get(AnalogInput.class, "pot");
	}

	@Override
	public void periodic() {
		ActiveOpMode.telemetry().addData("Angler Position", angler.getPosition());
		ActiveOpMode.telemetry().addData("pot voltage", potentiometer.getVoltage());

	}
	public double getAngle() {
		double voltage = potentiometer.getVoltage();
		double angle = (voltage / 3.3) * 2 * Math.PI;
		return 0;
	}
}
