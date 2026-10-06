package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.math.Pose;

public class PoseManager {

    private static Pose savedPose = null;

    public static void save(Pose pose) {savedPose = pose;}

    public static Pose get(Pose fallback) {return savedPose != null ? savedPose : fallback;}

    public static boolean hasPose() {return savedPose != null;}

    public static void clear() {savedPose = null;}

}
