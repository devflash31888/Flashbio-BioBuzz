package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
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

    public final DriveTrain drivetrain = new DriveTrain();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public final ShooterGate shooterGate = new ShooterGate();
    public final Vision vision = new Vision();

    public Robot() {
        shooter.setPoseSupplier(drivetrain::getPose);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return new HashSet<>(Arrays.asList(drivetrain, shooter, intake, shooterGate, vision));
    }
}