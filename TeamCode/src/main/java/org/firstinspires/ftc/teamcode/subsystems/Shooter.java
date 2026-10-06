package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.AllianceState;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import java.util.function.Supplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Shooter implements Mechanism {

    public enum Cell { LEFT, RIGHT }

    private final NextMotor motorA = new NextMotor("shooterA");
    private final NextMotor motorB = new NextMotor("shooterB");
    private final NextServo hood = new NextServo("hood");

    private final InterpLUT speedLUT = buildLut(RobotConfig.Shooter.shootSpeeds);
    private final InterpLUT hoodLUT = buildLut(RobotConfig.Shooter.shootHood);

    private Supplier<Pose> poseSupplier;
    private Cell cell = Cell.LEFT;
    private boolean enabled = false;
    private boolean autoAim = false;
    private double targetTps = 0;

    public Shooter() {
        motorA.setDirection(RobotConfig.Shooter.invertA
                ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
        motorB.follow(motorA, NextMotor.Direction.REVERSE);
    }

    private static InterpLUT buildLut(double[] values) {
        InterpLUT lut = new InterpLUT();
        double[] distances = RobotConfig.Shooter.shootDistances;
        for (int i = 0; i < distances.length; i++) lut.add(distances[i], values[i]);
        lut.createLUT();
        return lut;
    }

    public void setPoseSupplier(Supplier<Pose> supplier) { poseSupplier = supplier; }

    public void on()  { enabled = true; }
    public void off() { enabled = false; }
    public boolean isOn() { return enabled; }

    public void setAutoAim(boolean enable) { autoAim = enable; }

    public void setCell(Cell newCell) { cell = newCell; }
    public Cell getCell() { return cell; }


    private double[] goalPosition() {
        boolean red = AllianceState.current == Alliance.RED;
        boolean left = cell == Cell.LEFT;
        if (red) {
            return left ? new double[]{RobotConfig.Shooter.goalRedLeftX, RobotConfig.Shooter.goalRedLeftY}
                    : new double[]{RobotConfig.Shooter.goalRedRightX, RobotConfig.Shooter.goalRedRightY};
        }
        return left ? new double[]{RobotConfig.Shooter.goalBlueLeftX, RobotConfig.Shooter.goalBlueLeftY}
                : new double[]{RobotConfig.Shooter.goalBlueRightX, RobotConfig.Shooter.goalBlueRightY};
    }

    public double distanceToGoal() {
        Pose pose = poseSupplier.get();
        double[] goal = goalPosition();
        return Math.hypot(goal[0] - pose.x(), goal[1] - pose.y());
    }

    public void setDistance(double distance) {
        double[] distances = RobotConfig.Shooter.shootDistances;
        double clamped = Math.max(distances[0], Math.min(distances[distances.length - 1], distance));
        setTarget(speedLUT.get(clamped));
        setHood(hoodLUT.get(clamped));
    }

    public void setTarget(double tps) { targetTps = tps; }
    public double getTarget() { return targetTps; }

    public void setHood(double position) {
        hood.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    public double getVelocity() {
        return motorA.getEncoderVelocity().into(Units.RadiansPerSecond);
    }

    public double getError() { return targetTps - getVelocity(); }

    public boolean isReady() {
        return enabled && Math.abs(getError()) <= RobotConfig.Shooter.readyTolerance;
    }

    public Command spinUp() { return instant(this::on); }
    public Command stop()   { return instant(this::off); }

    @Override
    public void periodic() {
        motorA.getVelocityConstants().setKP(RobotConfig.Shooter.kP);
        motorA.getVelocityConstants().setKV(RobotConfig.Shooter.kV);
        motorA.getVelocityConstants().setKS(RobotConfig.Shooter.kS);

        if (autoAim && poseSupplier != null) {
            setDistance(distanceToGoal());
        }

        if (!enabled) {
            motorA.setThrottle(0.0);
            return;
        }

        if (getError() > RobotConfig.Shooter.boostThreshold) {
            motorA.setThrottle(1.0);
        } else {
            motorA.setVelocitySetpoint(Units.RadiansPerSecond.of(targetTps));
        }
    }
}