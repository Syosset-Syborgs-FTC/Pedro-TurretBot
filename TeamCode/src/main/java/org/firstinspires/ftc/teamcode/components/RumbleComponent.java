package org.firstinspires.ftc.teamcode.components;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class RumbleComponent implements Component {
    public static final RumbleComponent INSTANCE = new RumbleComponent();
    public boolean active = false;
    public void postInit() {
        active = false;
    }
    public void toggle() {
        active = !active;
    }
    public void preUpdate() {
        if (active) {
            ActiveOpMode.gamepad1().rumble(1.0, 1.0, 50);
        }
    }
}
