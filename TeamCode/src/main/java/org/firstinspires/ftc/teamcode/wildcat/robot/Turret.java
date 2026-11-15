package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {

    private final OpMode opMode;
    private final CRServoImplEx turretServo;
    // rev-encoder

    public Turret(OpMode opMode) {
        this.opMode = opMode;
        this.turretServo = this.opMode.hardwareMap.get(CRServoImplEx.class, "turret");
    }

    public void loop() {

        if(this.opMode.gamepad2.right_trigger > 0) {
            this.turretServo.setPower(this.opMode.gamepad2.right_trigger);
        } else if (this.opMode.gamepad2.left_trigger > 0) {
            this.turretServo.setPower(-this.opMode.gamepad2.left_trigger);
        } else {
            this.turretServo.setPower(0);
        }

        this.opMode.telemetry.addData("Turret Power", this.turretServo.getPower());
    }
}
