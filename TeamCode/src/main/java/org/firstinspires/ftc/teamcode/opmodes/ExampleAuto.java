package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "ExampleAuto", group = "opmodes")
public class ExampleAuto extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(56, 8, 0);
    private final Pose testePose = poseFactory.of(0, 0, 270);

    private Path test() {
        return line(startPose, testePose).linear(startPose, testePose);
    }

    private Command autoRotine() {
        return sequential(
                follow(follower, test())
        );
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        Scheduler.reset();
    }

    @Override
    public void start() {
        schedule(autoRotine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
