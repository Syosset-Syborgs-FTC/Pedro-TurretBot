package org.firstinspires.ftc.teamcode.subsytems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.interpolators.FirstOrderEMAParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
@Configurable
public class TurretAngleControl implements Subsystem {
	public static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0.0000000001, 0.03);
	public static FirstOrderEMAParameters emaParameters = new FirstOrderEMAParameters(0.3);
	public static final TurretAngleControl INSTANCE = new TurretAngleControl();
	AnalogInput potentiometer;
	MotorEx turret = new MotorEx("tu", 0.001).reversed();

	ServoEx angler = new ServoEx("an");
	public boolean followingAprilTag = false;
	public double turretTargetAngle = 0;

	ControlSystem turretControl = ControlSystem.builder()
			.posPid(pidCoefficients)
			.emaInterpolator(emaParameters)
			.build();

	@Override
	public void initialize() {
		turretTargetAngle = 0;
		turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		turret.zero();
		turretControl.setGoal(new KineticState());
		angler.setPosition(0);
//		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		potentiometer = ActiveOpMode.hardwareMap().get(AnalogInput.class, "pot");
		LimeLightAprilTag.INSTANCE.initialize();
	}

	@Override
	public void periodic() {
		Telemetry telemetry = ActiveOpMode.telemetry();
		telemetry.addData("Angler Position", angler.getPosition());
		telemetry.addData("pot voltage", potentiometer.getVoltage());
		turretControl.setGoal(new KineticState(turretTargetAngle));
		// output shaft counts / 537.7 -> output shaft revs / 1.295 ->
		double scalar = 1 / 537.7 * Math.PI * 30 / 120 * 2;
		double turretPower = turretControl.calculate(turret.getState().times(scalar));
		telemetry.addData("Turret Power", turretPower);
		double turretAngle = turret.getCurrentPosition() * scalar;
		LimeLightAprilTag.INSTANCE.setTurretAngle(turretAngle);
		LimeLightAprilTag.INSTANCE.periodic();

		turret.setPower(turretPower);

		Vector alignTarget = new Vector();
		alignTarget.setOrthogonalComponents(-72, 72);
		if (followingAprilTag) {
			double robotHeading = SensorFusion.INSTANCE.getPose().getHeading();
			setTurretAngle(robotHeading - alignTarget.minus(SensorFusion.INSTANCE.getPose().getAsVector()).getTheta());
		}
		telemetry.addData("Target Angle", Math.toDegrees(turretTargetAngle));
		telemetry.addData("Interpolated Target", Math.toDegrees(turretControl.getGoal().getPosition()));
		telemetry.addData("Turret Angle", Math.toDegrees(turretTargetAngle));
		telemetry.addData("Following apriltag", followingAprilTag);
	}
	public double getTurretAngle() {
		return turretTargetAngle;
	}
	public void setTurretAngle(double angle) {
		angle = wrapAngle(angle);
		this.turretTargetAngle = angle;
		turretControl.setGoal(new KineticState(angle, 0));
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
