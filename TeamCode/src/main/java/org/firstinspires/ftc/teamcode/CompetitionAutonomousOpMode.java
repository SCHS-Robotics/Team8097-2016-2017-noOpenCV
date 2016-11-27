package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public abstract class CompetitionAutonomousOpMode extends AutonomousOpMode {

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final double closeToWallDistance = 60;//centimeters
    final double veryCloseToWallDistance = 30;//centimeters
    final double closestToWallDistance = 10;//centimeters

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();
        loadTapeValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean run = true;
        while (opModeIsActive()) {
            if (run) {
                moveAcrossFieldDistance(DEFAULT_DIAGONAL_SPEED, 95 * Math.sqrt(2));
                while (getRangeDistance() > closeToWallDistance && opModeIsActive()) {
                    moveAlongStartWall(DEFAULT_SIDEWAYS_SPEED * 0.8);
                    logData("sensor distance", getRangeDistance());
                    updateTelemetry();
                }
                stopRobot();
                alignWithWall();
                findTapeInward();
                alignWithWall();
                pushCorrectButton();
                moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 90);
                findTapeInward();
                alignWithWall();
                pushCorrectButton();
                run = false;
            }

            idle();
        }
    }

    public void alignWithWall() throws InterruptedException {
        double angleOffset = determineAngleOffset();
        if (angleOffset > 2) {
            spinRightDegrees(DEFAULT_SPIN_SPEED, angleOffset);
        } else if (angleOffset < -2) {
            spinLeftDegrees(DEFAULT_SPIN_SPEED, -angleOffset);
        }
    }

    //These movements are with respect to the field. Different for red and blue because they mirror each other.
    public abstract void moveAcrossField(double power);

    public abstract void moveAcrossFieldDistance(double power, double centimeters) throws InterruptedException;

    public void moveAlongStartWall(double power) {
        goRight(power);
    }

    public void moveAlongStartWallDistance(double power, double centimeters) throws InterruptedException {
        goRightDistance(power, centimeters);
    }

    public abstract void moveAlongBeaconWall(double power);

    public abstract void moveAlongBeaconWallDistance(double power, double centimeters) throws InterruptedException;


    //These movements are with respect to the autonomous side of the robot.
    public void moveLeftSideForward(double power) {
        moveFrontWheelsRight(power);
    }

    public void moveLeftSideBackward(double power) {
        moveFrontWheelsLeft(power);
    }

    public void moveRightSideForward(double power) {
        moveBackWheelsRight(power);
    }

    public void moveRightSideBackward(double power) {
        moveBackWheelsLeft(power);
    }

    public void findTapeRight() {
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
            goBackward(DEFAULT_FORWARD_SPEED * 0.85);
        }
        stopRobot();
    }

    public void findTapeLeft() {
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
            goForward(DEFAULT_FORWARD_SPEED * 0.85);
        }
        stopRobot();
    }

    public abstract void findTapeInward();

    public abstract void findTapeOutward();

    public abstract void pushCorrectButton() throws InterruptedException;

    public void loadTapeValues() {
        frontTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("frontTapeValue", -1000) + FtcRobotControllerActivity.calibrationSP.getFloat("frontGroundValue", -1000)) / 2.0;
        if (frontTapeLowThreshold < 0) {
            frontTapeLowThreshold = 3;
        }
        backTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("backTapeValue", -1000) + FtcRobotControllerActivity.calibrationSP.getFloat("backGroundValue", -1000)) / 2.0;
        if (backTapeLowThreshold < 0) {
            backTapeLowThreshold = 3;
        }
    }
}