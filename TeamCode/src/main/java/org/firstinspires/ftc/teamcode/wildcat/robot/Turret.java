package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {

    private final static int PIPELINE = 0;

    // 54:10 Gear Ratio (5.4 Motor/Encoder Turns = 1 Full Turret Turn)
    public static final double GEAR_RATIO = 54.0 / 10.0;                 // 5.4:1
    public static final double ENCODER_CPR = 8192.0;                      // 8192 counts per encoder revolution
    public static final double TURRET_TICKS_PER_REV = ENCODER_CPR * GEAR_RATIO; // 44,236.8 ticks per full 360 deg turret turn
    public static final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0; // 122.88 ticks / degree

    public static final int CENTER_TICKS = 0;                             // Straight forward = 0 ticks (Magnetic switch)
    public static final int HALF_TURN_TICKS = (int) (180.0 * TICKS_PER_DEGREE); // 22,118 ticks (180 deg)
    public static final int FULL_TURN_TICKS = (int) (360.0 * TICKS_PER_DEGREE); // 44,236 ticks (360 deg)

    public static final int MIN_ALLOWED_TICKS = -HALF_TURN_TICKS;         // Left -180 degrees (-22,118 ticks)
    public static final int MAX_ALLOWED_TICKS = HALF_TURN_TICKS;          // Right +180 degrees (+22,118 ticks)

    // Camera offset: 88.5 mm = 3.48425 inches behind robot center
    public static final double LIMELIGHT_OFFSET_BEHIND_CENTER_INCHES = 88.5 / 25.4;

    // Alignment PID & Control parameters
    public static double KP = 0.035;
    public static double KI = 0.001;
    public static double KD = 0.003;
    public static double KP_ENCODER = 0.02; // Position Seeking PID gain
    public static double DEADBAND_DEG = 1.0;
    public static double MAX_TURRET_POWER = 0.8;
    public static double SWEEP_POWER = 0.25;
    public static double HOMING_POWER = 0.25; // Search power for magnetic switch

    // Distance Calculation parameters (inches & degrees)
    public static double CAMERA_HEIGHT_INCHES = 13.7795;
    public static double TARGET_HEIGHT_INCHES = 59.55;
    public static double CAMERA_MOUNT_ANGLE_DEG = 72.088;

    private final OpMode opMode;
    private final CRServoImplEx turretServo;
    private final DcMotorEx revEncoder;
    private final Limelight3A limelight;
    private final IMU imu;
    private final DigitalChannel megSwitch;

    private final ElapsedTime timer;
    private final ElapsedTime buttonTimer;

    private boolean autoAlignEnabled = false;
    private boolean targetVisible = false;
    private boolean isAligned = false;
    private boolean isHomed = false;

    private double txRaw = 0.0;
    private double txCenter = 0.0;
    private double ty = 0.0;
    private double distanceInches = 0.0;
    private int fiducialId = -1;

    private double lastError = 0.0;
    private double integral = 0.0;
    private double lastTime = 0.0;

    private int homingDirection = 1; // 1 for right, -1 for left
    private int homingStartPos = 0;

    public Turret(OpMode opMode) {
        this.opMode = opMode;
        this.timer = new ElapsedTime();
        this.buttonTimer = new ElapsedTime();

        this.turretServo = this.opMode.hardwareMap.get(CRServoImplEx.class, "turret");
        this.revEncoder = this.opMode.hardwareMap.get(DcMotorEx.class, "rev-encoder");
        this.revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.revEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = this.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        imu = this.opMode.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        this.megSwitch = this.opMode.hardwareMap.get(DigitalChannel.class, "magnetic");
        this.megSwitch.setMode(DigitalChannel.Mode.INPUT);

        isHomed = false;
        homingStartPos = revEncoder.getCurrentPosition();
    }

    private boolean isMagSwitchPressed() {
        return !this.megSwitch.getState();
    }

    /**
     * Homing via Magnetic Switch during init_loop():
     * Slowly sweeps right up to half turn (180 deg / 22118 ticks). If not triggered, sweeps left up to 360 deg (44236 ticks).
     * Resets revEncoder to 0 when magnetic switch is triggered at straight forward (0 deg).
     */
    public void initLoop() {
        if (isHomed) {
            this.opMode.telemetry.addData("Turret Homing", "HOMED OK at Straight Center! Pos: %d ticks (%.1f deg)",
                    revEncoder.getCurrentPosition(), getTurretRelativeAngleDeg());
            return;
        }

        // Check if magnetic switch is triggered (Straight Center = 0 deg)
        if (isMagSwitchPressed()) {
            this.turretServo.setPower(0.0);
            this.revEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.revEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            isHomed = true;
            this.opMode.telemetry.addData("Turret Homing", "COMPLETE! Magnetic Switch Triggered (Zeroed Straight Ahead).");
            return;
        }

        int currentPos = revEncoder.getCurrentPosition();
        int travelFromStart = currentPos - homingStartPos;

        // Step 1: Search right up to half turn (+22118 ticks = +180 deg)
        if (homingDirection == 1 && travelFromStart >= HALF_TURN_TICKS) {
            // Reached half turn right without finding switch -> reverse and search left up to full 360 deg (-44236 ticks)
            homingDirection = -1;
        } else if (homingDirection == -1 && travelFromStart <= -FULL_TURN_TICKS) {
            // Reached 360 deg search limit -> reverse back to right
            homingDirection = 1;
        }

        this.turretServo.setPower(homingDirection * HOMING_POWER);
        this.opMode.telemetry.addData("Turret Homing", "SEARCHING MAG SWITCH (%s)... Travel: %d / %d ticks",
                (homingDirection > 0 ? "RIGHT" : "LEFT"), travelFromStart, (homingDirection > 0 ? HALF_TURN_TICKS : -FULL_TURN_TICKS));
    }

    public void loop() {
        loop(null);
    }

    public void loop(Follower follower) {
        // If not homed yet, run homing step
        if (!isHomed) {
            initLoop();
            return;
        }

        // Toggle auto-align mode using Gamepad1 Y button with debounce
        if (buttonTimer.seconds() > 0.3 && this.opMode.gamepad2.y) {
            buttonTimer.reset();
            autoAlignEnabled = !autoAlignEnabled;
        }

        Pose robotPose = (follower != null) ? follower.pose() : new Pose(72, 72, 0);

        // Check Limelight results
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            targetVisible = true;
            txRaw = result.getTx();
            ty = result.getTy();

            // Calculate distance based on Limelight pitch angle (ty)
            double angleRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
            double tanVal = Math.tan(angleRad);
            if (tanVal > 0.01) {
                distanceInches = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / tanVal;
            }

            if (!result.getFiducialResults().isEmpty()) {
                fiducialId = result.getFiducialResults().get(0).getFiducialId();
            } else {
                fiducialId = -1;
            }

            // Calculate angle offset from detected AprilTag to Cluster Center (Hive Center)
            double offsetInches = getTagOffsetToClusterCenterInches(fiducialId);
            double offsetAngleDeg = (distanceInches > 0) ? Math.toDegrees(Math.atan2(offsetInches, distanceInches)) : 0.0;
            txCenter = txRaw + offsetAngleDeg;

            // Correct robot position drift using AprilTag location
            if (follower != null && fiducialId >= 30 && fiducialId <= 45 && distanceInches > 0) {
                correctRobotPose(follower, fiducialId, txRaw, distanceInches);
            }

        } else {
            targetVisible = false;
            isAligned = false;
            fiducialId = -1;
            txRaw = 0.0;
            txCenter = 0.0;
        }

        double manualRight = this.opMode.gamepad2.right_trigger; // Positive power = Turn Right
        double manualLeft = this.opMode.gamepad2.left_trigger;   // Negative power = Turn Left
        double turretPower = 0.0;

        // Manual override takes precedence over auto tracking & auto search
        if (manualRight > 0.05) {
            turretPower = manualRight; // Positive power rotates right
            isAligned = false;
        } else if (manualLeft > 0.05) {
            turretPower = -manualLeft; // Negative power rotates left
            isAligned = false;
        } else if (autoAlignEnabled) {
            if (targetVisible) {
                // PID Alignment control to drive txCenter angle to 0 (straight at Hive center)
                double currentTime = timer.seconds();
                double dt = currentTime - lastTime;
                if (dt <= 0.0 || dt > 0.2) dt = 0.02;

                double error = txCenter;
                if (Math.abs(error) <= DEADBAND_DEG) {
                    turretPower = 0.0;
                    isAligned = true;
                    integral = 0.0;
                } else {
                    isAligned = false;
                    integral += error * dt;
                    double derivative = (error - lastError) / dt;
                    turretPower = KP * error + KI * integral + KD * derivative;
                    turretPower = Math.max(-MAX_TURRET_POWER, Math.min(MAX_TURRET_POWER, turretPower));
                }

                lastError = error;
                lastTime = currentTime;
            } else if (follower != null) {
                // Smart Field-Oriented Search using revEncoder + Pedro Pathing Pose
                double[] targetHive = getTargetHiveCenter(robotPose, OpModeStorage.alliance);
                double targetX = targetHive[0];
                double targetY = targetHive[1];

                double dx = targetX - robotPose.x();
                double dy = targetY - robotPose.y();
                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                double robotHeadingDeg = Math.toDegrees(robotPose.heading());

                // Desired relative angle with respect to robot forward direction (-180 to +180 deg)
                double desiredTurretRelativeAngle = normalizeAngleDeg(fieldAngleDeg - robotHeadingDeg);

                // Convert desired relative angle to target encoder ticks (-22118 to +22118)
                // Center (Straight ahead) = 0 ticks
                double targetEncoderTicks = desiredTurretRelativeAngle * TICKS_PER_DEGREE;
                targetEncoderTicks = Math.max(MIN_ALLOWED_TICKS + 250, Math.min(MAX_ALLOWED_TICKS - 250, targetEncoderTicks));

                // Closed-loop encoder position tracking
                double currentTicks = revEncoder.getCurrentPosition();
                double encoderErrorTicks = targetEncoderTicks - currentTicks;
                double encoderErrorDeg = encoderErrorTicks / TICKS_PER_DEGREE;

                if (Math.abs(encoderErrorDeg) <= DEADBAND_DEG) {
                    turretPower = 0.0;
                } else {
                    turretPower = KP_ENCODER * encoderErrorDeg;
                    turretPower = Math.max(-SWEEP_POWER, Math.min(SWEEP_POWER, turretPower));
                }
                isAligned = false;
            } else {
                turretPower = 0.0;
                isAligned = false;
            }
        } else {
            turretPower = 0.0;
            isAligned = false;
        }

        // Apply power with soft limit protection (-22118 to +22118 ticks)
        setTurretPowerWithSoftLimits(turretPower);

        // Detailed Turret Tuning & Status Telemetry
        this.opMode.telemetry.addLine("--- TURRET TUNING & STATUS ---");
        this.opMode.telemetry.addData("Magnetic Switch Pressed", isMagSwitchPressed());
        this.opMode.telemetry.addData("Turret Homed", isHomed);
        this.opMode.telemetry.addData("Turret Mode", autoAlignEnabled ? "AUTO (G1 Y Toggle)" : "MANUAL (G2 Triggers)");
        this.opMode.telemetry.addData("Turret Servo Power", "%.3f", turretPower);
        this.opMode.telemetry.addData("REV Encoder Ticks", "%d / [-22118, +22118]", revEncoder.getCurrentPosition());
        this.opMode.telemetry.addData("Turret Relative Angle", "%.2f deg", getTurretRelativeAngleDeg());

        this.opMode.telemetry.addLine("--- LIMELIGHT TARGETING & PID ---");
        this.opMode.telemetry.addData("Target Visible", targetVisible);
        this.opMode.telemetry.addData("Turret Aligned to Hive", isAligned);
        this.opMode.telemetry.addData("AprilTag ID", fiducialId);
        this.opMode.telemetry.addData("Limelight txRaw", "%.2f deg", txRaw);
        this.opMode.telemetry.addData("Target txCenter Error", "%.2f deg", txCenter);
        this.opMode.telemetry.addData("Limelight ty", "%.2f deg", ty);
        this.opMode.telemetry.addData("Target Distance", "%.2f in", distanceInches);
        this.opMode.telemetry.addData("PID Gains (P/I/D)", "P=%.4f, I=%.4f, D=%.4f", KP, KI, KD);
    }

    /**
     * Soft limit protection: Prevents turret from rotating past -22118 ticks (Left -180 deg) or +22118 ticks (Right +180 deg)
     */
    private void setTurretPowerWithSoftLimits(double power) {
        int currentTicks = revEncoder.getCurrentPosition();
        double outputPower = power;

        if (currentTicks <= MIN_ALLOWED_TICKS + 100 && power < 0) {
            outputPower = 0.0; // Left -180 deg hard limit
        } else if (currentTicks >= MAX_ALLOWED_TICKS - 100 && power > 0) {
            outputPower = 0.0; // Right +180 deg hard limit
        }

        this.turretServo.setPower(outputPower);
    }

    /**
     * Calculates turret angle relative to Straight-Forward Zero (0 ticks = 0 deg)
     * Range: -180 deg (-22118 ticks) to +180 deg (+22118 ticks)
     */
    public double getTurretRelativeAngleDeg() {
        return revEncoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * Calculates total turret heading in global field coordinates
     */
    public double getTurretFieldAngleDeg(double robotHeadingRad) {
        return normalizeAngleDeg(Math.toDegrees(robotHeadingRad) + getTurretRelativeAngleDeg());
    }

    /**
     * Corrects Pedro Pathing robot pose using AprilTag field location and Limelight distance
     */
    private void correctRobotPose(Follower follower, int id, double tx, double distInches) {
        double tagX = getTagFieldX(id);
        double tagY = getTagFieldY(id);
        boolean facingNorth = isTagFacingNorth(id);

        Pose currentPose = follower.pose();
        double headingRad = currentPose.heading();

        // Total Limelight field orientation = Robot Heading + Turret Relative Angle + tx
        double totalCameraFieldAngleRad = headingRad + Math.toRadians(getTurretRelativeAngleDeg() + tx);

        double camX, camY;
        if (facingNorth) {
            camX = tagX - distInches * Math.sin(totalCameraFieldAngleRad);
            camY = tagY - distInches * Math.cos(totalCameraFieldAngleRad);
        } else {
            camX = tagX - distInches * Math.sin(totalCameraFieldAngleRad);
            camY = tagY + distInches * Math.cos(totalCameraFieldAngleRad);
        }

        // Account for camera offset (88.5 mm = 3.48425 in behind robot center)
        double correctedX = camX + LIMELIGHT_OFFSET_BEHIND_CENTER_INCHES * Math.cos(headingRad);
        double correctedY = camY + LIMELIGHT_OFFSET_BEHIND_CENTER_INCHES * Math.sin(headingRad);

        // Sanity check: Ensure corrected pose is within field boundary (0..144)
        if (correctedX >= 0.0 && correctedX <= 144.0 && correctedY >= 0.0 && correctedY <= 144.0) {
            follower.setPose(new Pose(correctedX, correctedY, headingRad));
        }
    }

    /**
     * Determines the target Hive center (X, Y) based on alliance and robot Y position
     */
    public static double[] getTargetHiveCenter(Pose robotPose, OpModeStorage.Alliance alliance) {
        double hiveX = (alliance == OpModeStorage.Alliance.RED) ? 60.0 : 84.0;
        double hiveY;

        if (robotPose.y() > 96.0) {
            hiveY = 96.0; // Target Back Hive (opposite audience)
        } else if (robotPose.y() < 48.0) {
            hiveY = 48.0; // Target Audience Hive
        } else {
            // Between 48 and 96: target whichever Hive is closer
            hiveY = (robotPose.y() > 72.0) ? 96.0 : 48.0;
        }

        return new double[]{hiveX, hiveY};
    }

    public static double getTagFieldX(int id) {
        switch (id) {
            case 30: case 34: return 53.50;
            case 31: case 35: return 57.25;
            case 32: case 36: return 62.75;
            case 33: case 37: return 67.00;
            case 38: case 42: return 77.50;
            case 39: case 43: return 81.25;
            case 40: case 44: return 86.75;
            case 41: case 45: return 91.00;
            default: return 72.0;
        }
    }

    public static double getTagFieldY(int id) {
        if (id >= 30 && id <= 33) return 96.0;  // Red Back
        if (id >= 34 && id <= 37) return 48.0;  // Red Audience
        if (id >= 38 && id <= 41) return 48.0;  // Blue Audience
        if (id >= 42 && id <= 45) return 96.0;  // Blue Back
        return 72.0;
    }

    public static boolean isTagFacingNorth(int id) {
        return (id >= 34 && id <= 41);
    }

    public double getTagOffsetToClusterCenterInches(int id) {
        if (id < 30) return 0.0;
        int index = (id - 30) % 4;
        switch (index) {
            case 0: return 6.50;   // Outer Left tag
            case 1: return 2.75;   // Inner Left tag
            case 2: return -2.75;  // Inner Right tag
            case 3: return -7.00;  // Outer Right tag
            default: return 0.0;
        }
    }

    private double normalizeAngleDeg(double angleDeg) {
        double angle = angleDeg;
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public boolean isAligned() {
        return isAligned;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public double getTx() {
        return txCenter;
    }

    public double getTy() {
        return ty;
    }

    public double getDistanceInches() {
        return distanceInches;
    }

    public int getFiducialId() {
        return fiducialId;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    public void setAutoAlignEnabled(boolean enabled) {
        this.autoAlignEnabled = enabled;
    }
}
