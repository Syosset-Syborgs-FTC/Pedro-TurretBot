package org.firstinspires.ftc.teamcode.teleop;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import static dev.nextftc.ftc.Gamepads.gamepad1;


import static dev.nextftc.ftc.Gamepads.gamepad2;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.photon.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;


import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.components.PanelsPacketComponent;
import org.firstinspires.ftc.teamcode.components.TelemetryComponent;
import org.firstinspires.ftc.teamcode.control.ShooterInterpolator.ShooterState;
import org.firstinspires.ftc.teamcode.control.ShooterInterpolator;
import org.firstinspires.ftc.teamcode.localizer.LimeLightAprilTag;
import org.firstinspires.ftc.teamcode.localizer.SensorFusion;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsytems.Ascent;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.ftc.components.LoopTimeComponent;
import dev.nextftc.hardware.impl.MotorEx;


//@Disabled
@TeleOp(name = "Test Teleop")
@Configurable
public class MotorTest extends NextFTCOpMode {


    private MotorEx flywheel, flywheel2;


    @Override
    public void onInit() {
        flywheel = new MotorEx("st");
        flywheel2 = new MotorEx("st2");
    }
    public void onWaitForStart() {


    }




    @Override
    public void onStartButtonPressed() {


    }


    @Override
    public void onUpdate() {
        flywheel.setPower(1);
        flywheel2.setPower(1);
    }
    public void onStop() {


    }
}