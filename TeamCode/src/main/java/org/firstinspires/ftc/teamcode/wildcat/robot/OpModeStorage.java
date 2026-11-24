package org.firstinspires.ftc.teamcode.wildcat.robot;

import com.pedropathing.math.Pose;

public class OpModeStorage {
    public enum Alliance {
        RED,
        BLUE,
    }

    // default pose to avoid null errors
    public static Pose autonomousEndPose = new Pose(0, 0, 0);
    public static Alliance alliance = Alliance.RED;
}