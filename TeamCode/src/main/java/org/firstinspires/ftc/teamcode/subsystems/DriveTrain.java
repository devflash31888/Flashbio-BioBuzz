package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.AllianceState;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.Mechanism;

public class DriveTrain implements Mechanism {

    private Follower follower;

    private double headingOffSet = 0;
    private boolean slowMode = false;

    public void init(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        headingOffSet = 0;
        slowMode = false;
    }

    public Follower follower() {return follower;}

    public Command teleopDrive(Gamepad g) {
        return infinite(() -> drive(-g.left_stick_y, g.left_stick_x, g.right_stick_x));
    }

    public void setSlowMode(boolean on) {slowMode = on;}

    private void drive(double forward, double lateral, double turn) {
        double f = shape(forward);
        double l = shape(lateral);
        double t = shape(turn) * RobotConfig.Drive.turnSensitivity;

        double scale = slowMode ? RobotConfig.Drive.slowModeFactor : 1.0;

        double effectiveHeading = getHeading() - headingOffSet - forwardHeading();
        DrivePowers powers = ManualDrive.fieldCentric(f * scale, l * scale, t * scale, effectiveHeading);

        follower.manual(powers);
    }

    public void resetHeading() {
        headingOffSet = getHeading() - forwardHeading();
    }

    public void relocalize() {
        Pose p;
        if(AllianceState.current == Alliance.RED) {
            p = new Pose(RobotConfig.Field.relocRedX, RobotConfig.Field.relocRedY,
                    Math.toRadians(RobotConfig.Field.relocRedHeadingDeg));
        } else {
            p = new Pose(RobotConfig.Field.relocBlueX, RobotConfig.Field.relocBlueY,
                    Math.toRadians(RobotConfig.Field.relocBlueHeadingDeg));
        }

        follower.setPose(p);
        headingOffSet = 0;
    }

    public void setPose(Pose pose) { follower.setPose(pose); }
    public Pose getPose()          { return follower.pose(); }
    public double getHeading()     { return follower.pose().heading(); }

    private double forwardHeading() {
        double deg = AllianceState.current == Alliance.RED
                ? RobotConfig.Field.forwardHeadingRedDeg
                : RobotConfig.Field.forwardHeadingBlueDeg;

        return Math.toRadians(deg);
    }

    private double shape(double input) {
        double deadzone = RobotConfig.Drive.stickDeadzone;
        double magnitude = Math.abs(input);

        if (magnitude < deadzone) return 0;

        magnitude = (magnitude - deadzone) / (1 - deadzone);
        return Math.copySign(Math.pow(magnitude, RobotConfig.Drive.stickExponent), input);
    }


    @Override
    public void periodic() {
        if(follower != null) follower.update();
    }

}
