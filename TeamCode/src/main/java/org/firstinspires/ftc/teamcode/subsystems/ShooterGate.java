package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class ShooterGate implements Mechanism {

    private final NextServo servo = new NextServo("gateShooter");

    private boolean open = false;
    private long lastChangeMs = 0;

    public void open() {
        if(!open) lastChangeMs = System.currentTimeMillis();
        open = true;
    }

    public void close() {
        if(open) lastChangeMs = System.currentTimeMillis();
        open = false;
    }

    public boolean isOpen() {return open;}

    public boolean isSettled() {
        return System.currentTimeMillis() - lastChangeMs >= RobotConfig.Gate.moveTimeMs;
    }

    public Command openGate() {return instant(this::open);}
    public Command closeGate() {return instant(this::close);}

    @Override
    public void periodic() {
        servo.setPosition(open ? RobotConfig.Gate.openPosition
                               : RobotConfig.Gate.closedPosition);
    }

}
