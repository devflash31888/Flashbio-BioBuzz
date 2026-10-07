package org.firstinspires.ftc.teamcode.config;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConfig {

    @Configurable
    public static class Drive {
        public static double stickDeadzone = 0.05;
        public static double stickExponent = 2.0;
        public static double slowModeFactor = 0.5;
        public static double turnSensitivity = 1.0;
    }

    @Configurable
    public static class Field {
        public static double forwardHeadingRedDeg = 0; //testar e ajustar
        public static double forwardHeadingBlueDeg = 0; //testar e ajustar

        public static double relocRedX = 0; //ajustar
        public static double relocRedY = 0; //ajustar
        public static double relocRedHeadingDeg = 0; //ajustar

        public static double relocBlueX = 0; //ajustar
        public static double relocBlueY = 0; //ajustar
        public static double relocBlueHeadingDeg = 0; //ajustar
    }

    @Configurable
    public static class Shooter {
        public static boolean invertA = false;
        public static double kP = 0.0; // ajustar
        public static double kV = 0.0057;
        public static double kS = 0.0; // ajustar
        public static double boostThreshold = 90;
        public static double readyTolerance = 30;
        public static double fixedVelocity = 1100;

        public static double[] shootDistances = {0, 0, 0, 0, 0};// ajustar
        public static double[] shootSpeeds = {0, 0, 0, 0, 0};// ajustar
        public static double[] shootHood = {0, 0, 0, 0, 0};// ajustar

        public static double goalRedLeftX = 0, goalRedLeftY = 0;// ajustar
        public static double goalRedRightX = 0, goalRedRightY = 0;// ajustar

        public static double goalBlueLeftX = 0, goalBlueLeftY = 0;// ajustar
        public static double goalBlueRightX = 0, goalBlueRightY = 0;// ajustar
    }

    @Configurable
    public static class Intake {
        public static boolean invert = false;
        public static double intakePower = 1.0;
        public static double reversePower = -1.0;
    }

    @Configurable
    public static class Gate {
        public static double openPosition = 0;//ajustar
        public static double closedPosition = 0;//ajustar
        public static double moveTimeMs = 250;//ajustar (tempo que o servo demora pra abrir)
    }

    @Configurable
    public static class Vision{
        public static int tagPipeline = 0;
        public static int pollenPipeline = 2;

        public static double servoTagsPosition = 0.8; //ajustar
        public static double servoPollenPosition = 0.2; //ajustar
        public static double servoSettleMs = 300; //ajustar

        public static int[] allowTagIdsBlue = {0, 0, 0, 0}; //ajustar
        public static int[] allowTagIdsRed = {0, 0, 0, 0}; //ajustar
        public static String pollenClassName = "Yellow";

        public static double maxStalenessMs = 100;
        public static double pollenStabelMs = 100;
        public static int pollRateHz = 100;
    }

    @Configurable
    public static class FlowerRemover {
        public static double downPosition = 0.8; //ajustar pos down
        public static double upPosition = 0.2; //ajustar pos default
        public static double moveTimeMs = 250; //ajustar (tempo que demora para descer)
        public static boolean invertRight = false;
    }
}
