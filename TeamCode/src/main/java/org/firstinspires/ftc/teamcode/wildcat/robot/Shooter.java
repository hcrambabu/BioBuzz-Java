package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private double MOTOR_TICKS = 28;
    private double MOTOR_RPS = 6000.0/60.0;
    private double MIN_SPEED = 0;
    private double MAX_SPEED = MOTOR_RPS * MOTOR_TICKS;
    private double SPEED_CHANGE = 50;
    private final OpMode opMode;
    private final DcMotorEx shooterMotor;
    private double speed = MIN_SPEED;

    private ElapsedTime elapsedTime;
    public Shooter(OpMode opMode) {
        this.opMode = opMode;
        this.elapsedTime = new ElapsedTime();
        this.shooterMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        if(elapsedTime.seconds() > 0.3) {
            elapsedTime.reset();
            if (this.opMode.gamepad2.left_bumper) {
                speed = speed - SPEED_CHANGE;
                speed = Math.max(MIN_SPEED, Math.min(MAX_SPEED, speed));
            } else if (this.opMode.gamepad2.right_bumper) {
                speed = speed + SPEED_CHANGE;
                speed = Math.max(MIN_SPEED, Math.min(MAX_SPEED, speed));
            }
        }
        this.shooterMotor.setVelocity(speed);

        this.opMode.telemetry.addData("Set Shooter Velocity", speed);
        this.opMode.telemetry.addData("Act Shooter Velocity", this.shooterMotor.getVelocity());
    }
}
