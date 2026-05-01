package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleHelper {
    private final Gamepad gamepad;
    private boolean active = false;

    public RumbleHelper(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void toggle() {
        active = !active;
    }

    public void update() {
        if (active) {
            // 1.0, 1.0 is max strength for both motors
            // 50ms ensures it stays vibrating until the next loop
            gamepad.rumble(1.0, 1.0, 50);
        }
    }

    public boolean isActive() {
        return active;
    }
}
