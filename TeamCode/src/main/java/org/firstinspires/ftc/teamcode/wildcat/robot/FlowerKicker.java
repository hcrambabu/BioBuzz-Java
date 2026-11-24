package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlowerKicker {

    private static final double MIN_POS = 0.25;
    private static final double MAX_POS = 0.8;

    private final OpMode opMode;
    private final ServoImplEx leftServo;
    private final ServoImplEx rightServo;

    private final ElapsedTime elapsedTime;
    private double position = MAX_POS;
    public FlowerKicker(OpMode opMode) {
        this.opMode = opMode;
        this.elapsedTime = new ElapsedTime();
        this.leftServo = this.opMode.hardwareMap.get(ServoImplEx.class, "left-flower");
        this.rightServo = this.opMode.hardwareMap.get(ServoImplEx.class, "right-flower");
    }

    public void loop() {
        if(elapsedTime.seconds() > 0.3) {
            elapsedTime.reset();
            if(this.opMode.gamepad2.right_bumper) {
                position = MIN_POS;
            } else if (this.opMode.gamepad2.left_bumper) {
                position = MAX_POS;
            }
        }
        this.leftServo.setPosition(position);
        this.rightServo.setPosition(position);

        this.opMode.telemetry.addData("Set Flower Pos", position);
        this.opMode.telemetry.addData("Act Flower Pos", this.leftServo.getPosition() +", "+ this.rightServo.getPosition());
    }
}
