package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public abstract class BeaconsAutonomous extends CompetitionAutonomous {

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final int closeToWallDistance = 30;//centimeters
    final int beforePushingButtonDistance = 16;//centimeters
    final int pushingButtonDistance = 12;//centimeters

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();
        loadTapeValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (shouldShoot()) {
            shoot();
            goBackwardDistance(DEFAULT_FORWARD_SPEED, 102.0 / 2);
            fixPosAfterShooting();
        } else {
            moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 102.0 / 2);
        }
        moveAlongStartWallDistance(DEFAULT_SIDEWAYS_SPEED, 102.0 / 2);
        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 102.0 / 2);
        goToBeaconWall(DEFAULT_SIDEWAYS_SPEED, closeToWallDistance);
        findTapeInward();
        alignWithWall();
        moveCorrectButtonFlap();
        pushButton();
        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 105);
        findTapeInward();
        alignWithWall();
        moveCorrectButtonFlap();
        pushButton();
        setTeleOpAngle();

        while (opModeIsActive()) {
            idle();
        }
    }

    public abstract boolean shouldShoot();

    public abstract void fixPosAfterShooting() throws InterruptedException;

    public void goToBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        while (getRangeDistance() > cmFromWall && opModeIsActive()) {
            moveAlongStartWall(speed);
        }
        stopRobot();
    }

    public void goAwayFromBeaconWall(double speed, int cmFromWall) throws InterruptedException {
        sleep(250);
        while (getRangeDistance() < cmFromWall && opModeIsActive()) {
            moveAlongStartWall(-speed);
        }
        stopRobot();
    }


    public void alignWithWall() throws InterruptedException {
//        double angleOffset = determineAngleOffset();
//        if (angleOffset > 2) {
//            spinRightDegrees(0.25, angleOffset);
//        } else if (angleOffset < -2) {
//            spinLeftDegrees(0.25, -angleOffset);
//        }
        goAwayFromBeaconWall(0.5, beforePushingButtonDistance);
        goToBeaconWall(0.5, beforePushingButtonDistance);
        stopRobot();
    }

    //These movements are with respect to the field. Different for red and blue because they mirror each other.
    public abstract void moveAcrossField(double power);

    public abstract void moveAcrossFieldDistance(double power, double centimeters) throws InterruptedException;

    public void moveAlongStartWall(double power) {
        goLeft(power);
    }

    public void moveAlongStartWallDistance(double power, double centimeters) throws InterruptedException {
        goLeftDistance(power, centimeters);
    }

    public abstract void moveAlongBeaconWall(double power);

    public abstract void moveAlongBeaconWallDistance(double power, double centimeters) throws InterruptedException;

    //These movements are with respect to the autonomous side of the robot.
    public void moveLeftSideForward(double power) {
        moveBackWheelsLeft(power);
    }

    public void moveLeftSideBackward(double power) {
        moveBackWheelsRight(power);
    }

    public void moveRightSideForward(double power) {
        moveFrontWheelsLeft(power);
    }

    public void moveRightSideBackward(double power) {
        moveFrontWheelsRight(power);
    }

    public void findTapeRight() throws InterruptedException {
        sleep(250);
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
            goForward(0.25);
        }
        stopRobot();
    }

    public void findTapeLeft() throws InterruptedException {
        sleep(250);
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            logData("light", frontTapeSensor.alpha());
            updateTelemetry();
            goBackward(0.25);
        }
        stopRobot();
    }

    public abstract void findTapeInward() throws InterruptedException;

    public abstract void findTapeOutward() throws InterruptedException;

    public abstract void moveCorrectButtonFlap() throws InterruptedException;

    public void pushButton() throws InterruptedException {
        goToBeaconWall(0.5, pushingButtonDistance);
        moveAlongStartWallDistance(0.5, 0.5);
        goAwayFromBeaconWall(0.5, beforePushingButtonDistance);
        resetButtonFlaps();
    }

    public void resetButtonFlaps() {
        leftFlapServo.setPosition(leftFlapInitPos);
        rightFlapServo.setPosition(rightFlapInitPos);
    }

    public abstract void setTeleOpAngle();

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