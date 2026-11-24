package org.firstinspires.ftc.teamcode.wildcat.robot;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Shooter {

    public enum BallType {
        POLLEN,  // Yellow ball (lighter weight)
        NECTAR,  // Red or Blue ball (heavier weight)
        UNKNOWN  // No ball detected / ambiguous
    }

    // goBILDA 5203 Series 1:1 Motor (6000 RPM) Specs
    // 28 Ticks / Revolution
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double MOTOR_NO_LOAD_RPM = 6000.0;
    public static final double THEORETICAL_MAX_VELOCITY = (MOTOR_NO_LOAD_RPM / 60.0) * MOTOR_TICKS_PER_REV; // 2800.0 ticks/sec

    private static final double MIN_SPEED = 0.0;
    private static final double MAX_SPEED = THEORETICAL_MAX_VELOCITY;
    private static final double SPEED_CHANGE = 50.0;
    public static double VELOCITY_TOLERANCE = 80.0; // Ticks/sec tolerance for speed ready (~170 RPM)

    // Pollen speed curve (Min/Max Distance & Min/Max Speed)
    public static double POLLEN_MIN_DIST = 20.0;    // inches
    public static double POLLEN_MAX_DIST = 120.0;   // inches
    public static double POLLEN_MIN_SPEED = 1000.0; // ticks/sec (~2140 RPM)
    public static double POLLEN_MAX_SPEED = 2200.0; // ticks/sec (~4714 RPM)

    // Nectar speed curve (Heavier ball -> higher required speed, capped at realistic max ~2500 ticks/sec = 5357 RPM)
    public static double NECTAR_MIN_DIST = 20.0;    // inches
    public static double NECTAR_MAX_DIST = 120.0;   // inches
    public static double NECTAR_MIN_SPEED = 1400.0; // ticks/sec (~3000 RPM)
    public static double NECTAR_MAX_SPEED = 2500.0; // ticks/sec (~5357 RPM)

    // Velocity PIDF Parameters for Flywheel Recovery & Control
    public static double KF = 1.0 / THEORETICAL_MAX_VELOCITY; // ~0.0003571
    public static double KP = 0.0015;
    public static double KI = 0.0001;
    public static double KD = 0.00005;
    public static double MAX_INTEGRAL = 0.2;

    private final OpMode opMode;
    private final DcMotorEx shooterMotor;
    private final ServoImplEx shooterServo;
    private NormalizedColorSensor colorSensor;

    private final ElapsedTime elapsedTime;
    private final ElapsedTime buttonTimer;
    private final ElapsedTime pidTimer;

    private double manualSpeed = 0.0; // Default OFF (0 RPM) on startup
    private double targetSpeed = MIN_SPEED;
    private boolean autoSpeedEnabled = false;
    private BallType detectedBallType = BallType.UNKNOWN;

    private double currentDistanceInches = 0.0;
    private boolean hasTarget = false;

    private double lastError = 0.0;
    private double integral = 0.0;
    private double lastPidTime = 0.0;
    private double motorPower = 0.0;

    public Shooter(OpMode opMode) {
        this.opMode = opMode;
        this.elapsedTime = new ElapsedTime();
        this.buttonTimer = new ElapsedTime();
        this.pidTimer = new ElapsedTime();

        this.shooterMotor = this.opMode.hardwareMap.get(DcMotorEx.class, "shooter");
        this.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.shooterServo = this.opMode.hardwareMap.get(ServoImplEx.class, "shooter-servo");
        this.shooterServo.setPosition(0);

        try {
            this.colorSensor = this.opMode.hardwareMap.get(NormalizedColorSensor.class, "color");
            this.colorSensor.setGain(15.0f);
        } catch (Exception e) {
            this.colorSensor = null;
        }
    }

    public void setTargetDistance(double distanceInches, boolean hasTarget) {
        this.currentDistanceInches = distanceInches;
        this.hasTarget = hasTarget;
    }

    public void loop() {
        // Mode toggle: Gamepad1 X toggles auto/manual speed mode
        if (buttonTimer.seconds() > 0.3 && this.opMode.gamepad2.x) {
            buttonTimer.reset();
            autoSpeedEnabled = !autoSpeedEnabled;
        }

        // Fine tuning speed adjustments with Gamepad1 bumpers
        if (elapsedTime.seconds() > 0.15) {
            if (this.opMode.gamepad1.left_bumper) {
                elapsedTime.reset();
                manualSpeed = Math.max(MIN_SPEED, manualSpeed - SPEED_CHANGE);
            } else if (this.opMode.gamepad1.right_bumper) {
                elapsedTime.reset();
                manualSpeed = Math.min(MAX_SPEED, manualSpeed + SPEED_CHANGE);
            }
        }

        // Indexer Servo position control (Gamepad2 A = Feed/Fire, B = Reset)
        if (this.opMode.gamepad2.a) {
            this.shooterServo.setPosition(1.0);
        } else if (this.opMode.gamepad2.b) {
            this.shooterServo.setPosition(0.0);
        }

        // Detect ball type using Color Sensor
        detectedBallType = detectBallType();

        // Compute target speed based on mode
        if (autoSpeedEnabled && hasTarget && currentDistanceInches > 0) {
            targetSpeed = calculateAutoSpeed(currentDistanceInches, detectedBallType);
        } else {
            targetSpeed = manualSpeed;
        }

        // Custom Velocity PIDF Control loop
        double currentTime = pidTimer.seconds();
        double elapsed = currentTime - lastPidTime;
        double dt = (elapsed > 0.0 && elapsed <= 0.2) ? elapsed : 0.02;

        double actualVelocity = this.shooterMotor.getVelocity();
        double error = targetSpeed - actualVelocity;

        if (targetSpeed <= 0.0) {
            motorPower = 0.0;
            integral = 0.0;
        } else {
            double feedforward = KF * targetSpeed;
            integral += error * dt;

            // Anti-windup integral clamp
            double maxI = (KI > 0) ? MAX_INTEGRAL / KI : 0.0;
            integral = Math.max(-maxI, Math.min(maxI, integral));

            double derivative = (error - lastError) / dt;
            motorPower = feedforward + (KP * error) + (KI * integral) + (KD * derivative);
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));
        }

        lastError = error;
        lastPidTime = currentTime;

        this.shooterMotor.setPower(motorPower);

        // Detailed Shooter Flywheel Telemetry
        this.opMode.telemetry.addLine("--- SHOOTER FLYWHEEL & PIDF ---");
        this.opMode.telemetry.addData("Shooter Mode", autoSpeedEnabled ? "AUTO (G1 X Toggle)" : "MANUAL (G1 Bumpers)");
        this.opMode.telemetry.addData("Detected Ball", detectedBallType.name());
        this.opMode.telemetry.addData("Target Speed", "%.1f ticks/s (%.1f RPM)", targetSpeed, (targetSpeed / MOTOR_TICKS_PER_REV) * 60.0);
        this.opMode.telemetry.addData("Actual Speed", "%.1f ticks/s (%.1f RPM)", actualVelocity, (actualVelocity / MOTOR_TICKS_PER_REV) * 60.0);
        this.opMode.telemetry.addData("Velocity Error", "%.1f ticks/s", error);
        this.opMode.telemetry.addData("Motor Power Output", "%.2f", motorPower);
        this.opMode.telemetry.addData("Shooter Speed Ready", isSpeedReady());
    }

    private BallType detectBallType() {
        if (colorSensor == null) {
            return BallType.UNKNOWN;
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // If brightness or saturation is too low, no ball present
        if (value < 0.05 || saturation < 0.2) {
            return BallType.POLLEN;
        }

        // Pollen: Yellow ball (Hue approx 35 to 85)
        if (hue >= 35.0f && hue <= 85.0f) {
            return BallType.POLLEN;
        }
        // Nectar: Red ball (Hue 0 to 25 or 330 to 360) or Blue ball (Hue 180 to 260)
        if ((hue >= 0.0f && hue <= 25.0f) || (hue >= 330.0f && hue <= 360.0f) || (hue >= 180.0f && hue <= 260.0f)) {
            return BallType.NECTAR;
        }

        return BallType.POLLEN;
    }

    public double calculateAutoSpeed(double distanceInches, BallType ballType) {
        double minDist = (ballType == BallType.NECTAR) ? NECTAR_MIN_DIST : POLLEN_MIN_DIST;
        double maxDist = (ballType == BallType.NECTAR) ? NECTAR_MAX_DIST : POLLEN_MAX_DIST;
        double minSpeed = (ballType == BallType.NECTAR) ? NECTAR_MIN_SPEED : POLLEN_MIN_SPEED;
        double maxSpeed = (ballType == BallType.NECTAR) ? NECTAR_MAX_SPEED : POLLEN_MAX_SPEED;

        double clampedDist = Math.max(minDist, Math.min(maxDist, distanceInches));
        double ratio = (clampedDist - minDist) / (maxDist - minDist);

        return minSpeed + ratio * (maxSpeed - minSpeed);
    }

    public boolean isSpeedReady() {
        return Math.abs(this.shooterMotor.getVelocity() - targetSpeed) < VELOCITY_TOLERANCE;
    }

    public BallType getDetectedBallType() {
        return detectedBallType;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public boolean isAutoSpeedEnabled() {
        return autoSpeedEnabled;
    }

    public void setAutoSpeedEnabled(boolean enabled) {
        this.autoSpeedEnabled = enabled;
    }
}
