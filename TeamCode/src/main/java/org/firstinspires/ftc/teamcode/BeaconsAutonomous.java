package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public abstract class BeaconsAutonomous extends CompetitionAutonomous {

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final int closeToWallDistance = 25;//centimeters
    final int beforePushingButtonDistance = 10;//centimeters

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
        goForwardDistance(0.5, 1);
        findTapeLeft();
        pushButton();
        moveAlongStartWallDistance(-DEFAULT_SIDEWAYS_SPEED, 15);
        moveAlongBeaconWallDistance(DEFAULT_FORWARD_SPEED, 105);
        findTapeInward();
        alignWithWall();
        goForwardDistance(0.5, 1);
        findTapeLeft();
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

    public void findTapeRight() throws InterruptedException {
        sleep(250);
        boolean firstTime = true;
        while ((frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (!firstTime) {
                goBackwardDistance(0.25, 5);
            }
            firstTime = false;
            while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                logData("light", frontTapeSensor.alpha());
                updateTelemetry();
                goForward(0.25);
            }
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveLeftWheelsForward(0.25);
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveRightWheelsForward(0.25);
                }
            }
        }
        stopRobot();
    }

    public void findTapeLeft() throws InterruptedException {
        sleep(250);
        boolean firstTime = true;
        while ((frontTapeSensor.alpha() < frontTapeLowThreshold || backTapeSensor.alpha() < backTapeLowThreshold) && opModeIsActive()) {
            if (!firstTime) {
                goForwardDistance(0.25, 5);
            }
            firstTime = false;
            while (frontTapeSensor.alpha() < frontTapeLowThreshold && backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                logData("light", frontTapeSensor.alpha());
                updateTelemetry();
                goBackward(0.25);
            }
            if (frontTapeSensor.alpha() < frontTapeLowThreshold) {
                while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveLeftWheelsBackward(0.25);
                }
            } else if (backTapeSensor.alpha() < backTapeLowThreshold) {
                while (backTapeSensor.alpha() < backTapeLowThreshold && opModeIsActive()) {
                    logData("light", frontTapeSensor.alpha());
                    updateTelemetry();
                    moveRightWheelsBackward(0.25);
                }
            }
        }
        stopRobot();
    }

    public abstract void findTapeInward() throws InterruptedException;

    public abstract void findTapeOutward() throws InterruptedException;

    public abstract void moveCorrectButtonFlap() throws InterruptedException;

    public void pushButton() throws InterruptedException {
//        do {
        moveCorrectButtonFlap();
        sleep(250);
        resetButtonFlaps();
//            sleep(250);
//        } while (!buttonIsPressed());
    }

    public boolean buttonIsPressed() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        if (Math.abs(leftBlue - rightBlue) < 5 && Math.abs(leftRed - rightRed) < 5) {
            return true;
        }
        return false;
    }

    public void resetButtonFlaps() {
        leftFlapServo.setPosition(leftFlapInitPos);
        rightFlapServo.setPosition(rightFlapInitPos);
    }

    public abstract void setTeleOpAngle();

    public void loadTapeValues() {
        double frontGround = FtcRobotControllerActivity.calibrationSP.getFloat("frontGroundValue", -1000);
        double frontDiff = FtcRobotControllerActivity.calibrationSP.getFloat("frontTapeValue", -1000) - frontGround;
        frontTapeLowThreshold = frontGround + frontDiff * 0.65;
        if (frontTapeLowThreshold < 0) {
            frontTapeLowThreshold = 20;
        }
        double backGround = FtcRobotControllerActivity.calibrationSP.getFloat("backGroundValue", -1000);
        double backDiff = FtcRobotControllerActivity.calibrationSP.getFloat("backTapeValue", -1000) - backGround;
        backTapeLowThreshold = backGround + backDiff * 0.65;
        if (backTapeLowThreshold < 0) {
            backTapeLowThreshold = 20;
        }
    }
}