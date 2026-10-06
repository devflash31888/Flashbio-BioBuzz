package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {

    private enum Mode { OFF, IN, OUT }

    private final NextMotor motor = new NextMotor("intake");

    private Mode mode = Mode.OFF;

    public Intake() {
        motor.setDirection(RobotConfig.Intake.invert
                ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);
    }

    public void intake()  { mode = Mode.IN; }
    public void reverse() { mode = Mode.OUT; }
    public void stop()    { mode = Mode.OFF; }

    public boolean isRunning() { return mode != Mode.OFF; }

    public Command runIntake()  { return instant(this::intake); }
    public Command runReverse() { return instant(this::reverse); }
    public Command stopIntake() { return instant(this::stop); }

    @Override
    public void periodic() {
        switch (mode) {
            case IN:
                motor.setThrottle(RobotConfig.Intake.intakePower);
                break;
            case OUT:
                motor.setThrottle(RobotConfig.Intake.reversePower);
                break;
            default:
                motor.setThrottle(0.0);
        }
    }
}