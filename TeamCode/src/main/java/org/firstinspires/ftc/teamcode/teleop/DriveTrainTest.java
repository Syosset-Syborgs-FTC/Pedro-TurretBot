package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static dev.nextftc.ftc.Gamepads.*;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp
@Disabled
public class DriveTrainTest extends NextFTCOpMode {
	MotorEx fl = new MotorEx("fl");
	MotorEx fr = new MotorEx("fr");
	MotorEx bl = new MotorEx("bl");
	MotorEx br = new MotorEx("br");
	public DriveTrainTest() {
		addComponents(BindingsComponent.INSTANCE);
	}

	@Override
	public void onInit() {

	}

	@Override
	public void onStartButtonPressed() {
		gamepad1().y().whenTrue(new SetPower(fl, 1)).whenFalse(new SetPower(fl, 0));
		gamepad1().a().whenTrue(new SetPower(fr, 1)).whenFalse(new SetPower(fr, 0));
		gamepad1().x().whenTrue(new SetPower(bl, 1)).whenFalse(new SetPower(bl, 0));
		gamepad1().b().whenTrue(new SetPower(br, 1)).whenFalse(new SetPower(br, 0));

	}
}
