package org.firstinspires.ftc.teamcode.wildcat.opmode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "WildCat: TeleopWithBreaks", group = "WildCat")
public class TeleOpWithBreaks extends CommonTeleOp{

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        super.loop();
    }
}
