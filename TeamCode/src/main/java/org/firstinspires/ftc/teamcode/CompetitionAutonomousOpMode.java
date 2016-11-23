package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public abstract class CompetitionAutonomousOpMode extends AutonomousOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    final double closeToWallDistance = 60;//centimeters
    final double veryCloseToWallDistance = 30;//centimeters
    final double closestToWallDistance = 10;//centimeters

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allInit();
        loadTapeValues();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        boolean run = true;
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if (run) {
                moveAcrossFieldDistance(DEFAULT_DIAGONAL_POWER, 95 * Math.sqrt(2));
                while ((getLeftRangeDistance() > closeToWallDistance/* || getRightRangeDistance() > closeToWallDistance*/) && opModeIsActive()) {
                    moveAlongStartWall(DEFAULT_SIDEWAYS_POWER / 4.0);
                    telemetry.addData("left", getLeftRangeDistance());
                    telemetry.addData("right", getRightRangeDistance());
                    telemetry.update();
                }
                stopRobot();
                alignWithWall();
                findTapeInward();
                alignWithWall();
                pushCorrectButton();
                moveAlongBeaconWallDistance(DEFAULT_FORWARD_POWER, 90);
                findTapeInward();
                alignWithWall();
                pushCorrectButton();
                run = false;
            }

            idle();
        }
    }

    public void alignWithWall() {
        //??????
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

    public void findTapeRight() {
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            telemetry.addData("light", frontTapeSensor.alpha());
            telemetry.update();
            goForward(DEFAULT_FORWARD_POWER / 2.0);
        }
        stopRobot();
    }

    public void findTapeLeft() {
        while (frontTapeSensor.alpha() < frontTapeLowThreshold && opModeIsActive()) {
            telemetry.addData("light", frontTapeSensor.alpha());
            telemetry.update();
            goBackward(DEFAULT_FORWARD_POWER / 2.0);
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

    private void testApproachWall() throws InterruptedException {
        while ((getLeftRangeDistance() > closeToWallDistance && getRightRangeDistance() > closeToWallDistance) && opModeIsActive()) {
            moveAlongStartWall(DEFAULT_SIDEWAYS_POWER / 4.0);
            telemetry.addData("left", getLeftRangeDistance());
            telemetry.addData("right", getRightRangeDistance());
            telemetry.update();
        }
        stopRobot();
        sleep(1000);
        while ((getLeftRangeDistance() > veryCloseToWallDistance && getRightRangeDistance() > veryCloseToWallDistance) && opModeIsActive()) {
            moveAlongStartWall(DEFAULT_SIDEWAYS_POWER / 4.0);
            telemetry.addData("left", getLeftRangeDistance());
            telemetry.addData("right", getRightRangeDistance());
            telemetry.update();
        }
        stopRobot();
        sleep(1000);
        while ((getLeftRangeDistance() > closestToWallDistance && getRightRangeDistance() > closestToWallDistance) && opModeIsActive()) {
            moveAlongStartWall(DEFAULT_SIDEWAYS_POWER / 4.0);
            telemetry.addData("left", getLeftRangeDistance());
            telemetry.addData("right", getRightRangeDistance());
            telemetry.update();
        }
        stopRobot();
        sleep(1000);
        while (getLeftRangeDistance() > closestToWallDistance || getRightRangeDistance() > closestToWallDistance && opModeIsActive()) {
            while (getLeftRangeDistance() > closestToWallDistance && opModeIsActive()) {
                moveLeftSideForward(DEFAULT_SIDEWAYS_POWER / 4.0);
            }
            while (getRightRangeDistance() > closestToWallDistance && opModeIsActive()) {
                moveRightSideForward(DEFAULT_SIDEWAYS_POWER / 4.0);
            }
        }

    }
}