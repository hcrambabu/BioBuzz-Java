package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InTake {
    private final OpMode opMode;
    private final DcMotorEx leftIntakeMotor; // Controls front intake wheels
    private final DcMotorEx rightIntakeMotor; // Controls right intake wheels
    public InTake(OpMode opMode) {
        this.opMode = opMode;
        this.leftIntakeMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "left-intake");
        this.rightIntakeMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "right-intake");

        this.leftIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        this.leftIntakeMotor.setPower(this.opMode.gamepad2.left_stick_y);
        this.rightIntakeMotor.setPower(this.opMode.gamepad2.right_stick_y);
    }
}
