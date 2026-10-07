package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerRemover;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterGate;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

public class Robot implements NextRobot {

    DriveTrain drivetrain = new DriveTrain();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    ShooterGate shooterGate = new ShooterGate();
    Vision vision = new Vision();
    FlowerRemover remover = new FlowerRemover();

    public Robot() {
        shooter.setPoseSupplier(drivetrain::getPose);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(drivetrain, shooter, intake, shooterGate, vision, remover);
    }
}