package org.firstinspires.ftc.teamcode.wildcat.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.wildcat.robot.FlowerKicker;
import org.firstinspires.ftc.teamcode.wildcat.robot.InTake;
import org.firstinspires.ftc.teamcode.wildcat.robot.OpModeStorage;
import org.firstinspires.ftc.teamcode.wildcat.robot.Shooter;
import org.firstinspires.ftc.teamcode.wildcat.robot.Turret;

public class CommonTeleOp extends OpMode {

    protected Follower follower;
    protected InTake intake;
    protected Shooter shooter;
    protected Turret turret;
    protected FlowerKicker flowerKicker;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        this.intake = new InTake(this);
        this.shooter = new Shooter(this);
        this.turret = new Turret(this);
        this.flowerKicker = new FlowerKicker(this);
    }

    @Override
    public void init_loop() {
        // Continuously runs homing routine while robot is waiting in INIT phase
        this.turret.initLoop();
    }

    @Override
    public void start() {
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        this.intake.loop();

        // Pass follower to Turret for smart pose-based alignment & localization correction
        this.turret.loop(this.follower);

        this.shooter.setTargetDistance(this.turret.getDistanceInches(), this.turret.isTargetVisible());
        this.shooter.loop();

        this.flowerKicker.loop();

        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Alliance", OpModeStorage.alliance);
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
    }
}
