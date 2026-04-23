package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Common;
import org.firstinspires.ftc.teamcode.subsytems.Shooter;
import org.firstinspires.ftc.teamcode.subsytems.TurretAngleControl;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;

@Autonomous(group = "Auton")
public class Auton extends SyborgsAutonBase {

    @Override
    public void onInit() {
        super.onInit();
    }

    @Override
    public void onStartButtonPressed() {
        Common.alliance = Common.Alliance.Blue;

        // Start Flywheel spinning, but keep Gate CLOSED for now
        Shooter.INSTANCE.setTargetVelocity(1400);
        TurretAngleControl.INSTANCE.setAnglerPosition(0);
        Shooter.INSTANCE.setShooting(false);

        new SequentialGroup(
                // 1. WAIT for 5 seconds (Robot does nothing)
                new Delay(5),

                new InstantCommand(() -> {
                    // 2. NOW open gate and start intake at 0.5 power
                    Shooter.INSTANCE.setShooting(true);
                    Shooter.INSTANCE.setIntakeState(0.5);
                }),

                // 3. RUN for 10 seconds
                new Delay(10),

                new InstantCommand(() -> {
                    // 4. STOP everything
                    Shooter.INSTANCE.setIntakeState(0);
                    Shooter.INSTANCE.setShooting(false);
                })
        ).schedule();
    }
}