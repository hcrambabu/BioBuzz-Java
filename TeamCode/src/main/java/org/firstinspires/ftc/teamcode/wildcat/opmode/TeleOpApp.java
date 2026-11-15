package org.firstinspires.ftc.teamcode.wildcat.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "WildCat: Teleop", group = "WildCat")
public class TeleOpApp extends CommonTeleOp {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
        super.loop();
    }
}
