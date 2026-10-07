package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class FlowerRemover implements Mechanism {

    private final NextServo servoLeft = new NextServo("flowerServoLeft");
    private final NextServo servoRight = new NextServo("flowerServoRight");

    private boolean down = false;
    private long lastChangeMs = 0;

    public void down() {
        if(down) {
            lastChangeMs = System.currentTimeMillis();
        }
        down = true;
    }

    public void up() {
        if(!down) {
            lastChangeMs = System.currentTimeMillis();
        }
        down = false;
    }

    public boolean isDown() {
        return down;
    }

    public boolean isSettled() {
        return System.currentTimeMillis() - lastChangeMs >= RobotConfig.FlowerRemover.moveTimeMs;
    }

    public Command setDown() {return instant(this::down);}
    public Command setUp() {return instant(this::up);}

    @Override
    public void periodic() {
        double position = down ? RobotConfig.FlowerRemover.downPosition
                               : RobotConfig.FlowerRemover.upPosition;

        servoLeft.setPosition(position);

        double rightPos = RobotConfig.FlowerRemover.invertRight ? (1.0 - position) : position;
        servoRight.setPosition(rightPos);
    }

}
