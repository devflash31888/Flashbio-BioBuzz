package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotConfig;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Vision implements Mechanism {

    public enum Mode {TAGS, POLLENS}

    private final NextServo servo = new NextServo("LLServo");
    private Limelight3A limelight3A;

    private Mode mode = Mode.TAGS;
    private long modeChangedMs = 0;

    private boolean tagVisible = false;
    private int tagId = -1;
    private double tagTx = 0;
    private double tagDistanceInches = 0;

    private boolean pollenVisible = false;
    private double pollenTx = 0;
    private double pollenTy = 0;
    private double getPollenTy = 0;
    private long pollenSeenSinceMs = -1;

    public void init(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(RobotConfig.Vision.pollRateHz);
        limelight3A.start();
        applyMode(Mode.TAGS);
    }

    public void setMode(Mode newMode) {
        if (newMode == mode) return;
        applyMode(newMode);
    }

    private void applyMode (Mode newMode) {
        mode = newMode;
        modeChangedMs = System.currentTimeMillis();
        clearReadings();
        limelight3A.pipelineSwitch(expectedPipeline());
    }

    public Mode getMode() {return mode;}

    private int expectedPipeline() {
        return mode == Mode.TAGS ? RobotConfig.Vision.tagPipeline : RobotConfig.Vision.pollenPipeline;
    }

    public boolean isModeReady() {
        return System.currentTimeMillis() - modeChangedMs >= RobotConfig.Vision.servoSettleMs;
    }

    public Command lookAtTags() {return instant(() -> setMode(Mode.TAGS));}
    public Command lookAtPollens() {return instant(() -> setMode(Mode.POLLENS));}

    public boolean hasTag() {return tagVisible;}
    public int getTagId() {return tagId;}
    public double getTagTx() {return tagTx;}
    public double getTagDistanceInches() {return tagDistanceInches;}

    public boolean hasPollen() {return pollenVisible;}
    public double getPollenTx() {return pollenTx;}
    public double getGetPollenTy() {return pollenTy;}

    public boolean hasStablePollen() {
        return pollenVisible && pollenSeenSinceMs >= 0 && System.currentTimeMillis() - pollenSeenSinceMs >= RobotConfig.Vision.pollenStabelMs;
    }

    private void clearReadings() {
        tagVisible = false;
        tagId = -1;
        pollenVisible = false;
        pollenSeenSinceMs = -1;
    }

    private static boolean isAllowedTag(int id) {
        for(int allowed : RobotConfig.Vision.allowTagIds) {
            if(allowed == id) return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        if (limelight3A == null) return;

        servo.setPosition(mode == Mode.TAGS
                ? RobotConfig.Vision.servoTagsPosition
                : RobotConfig.Vision.servoPollenPosition);

        clearReadings();

        if (!isModeReady()) return;

        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getPipelineIndex() != expectedPipeline()) return;
        if (result.getStaleness() > RobotConfig.Vision.maxStalenessMs) return;

        if (mode == Mode.TAGS) {
            readTags(result);
        } else {
            readPollens(result);
        }
    }

    private void readTags(LLResult result) {
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (!isAllowedTag(tag.getFiducialId())) continue;

            tagVisible = true;
            tagId = tag.getFiducialId();
            tagTx = tag.getTargetXDegrees();

            Position position = tag.getTargetPoseCameraSpace().getPosition();
            double x = position.unit.toInches(position.x);
            double y = position.unit.toInches(position.y);
            double z = position.unit.toInches(position.z);
            tagDistanceInches = Math.sqrt(x * x + y * y + z * z);
            return;
        }
    }

    private void readPollens(LLResult result) {
        LLResultTypes.DetectorResult closest = null;
        for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
            if (!RobotConfig.Vision.pollenClassName.equals(detection.getClassName())) continue;
            // Mais próximo = menor ty (mais abaixo na imagem), supondo a câmera olhando para frente e para baixo
            if (closest == null || detection.getTargetYDegrees() < closest.getTargetYDegrees()) {
                closest = detection;
            }
        }

        if (closest == null) return;

        pollenVisible = true;
        pollenTx = closest.getTargetXDegrees();
        pollenTy = closest.getTargetYDegrees();
        if (pollenSeenSinceMs < 0) pollenSeenSinceMs = System.currentTimeMillis();
    }
}
