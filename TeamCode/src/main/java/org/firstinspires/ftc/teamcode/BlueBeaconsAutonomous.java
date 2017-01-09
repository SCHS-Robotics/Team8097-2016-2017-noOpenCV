package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Beacons Autonomous", group = "OpMode")
public class BlueBeaconsAutonomous extends BeaconsAutonomous {

    @Override
    public boolean shouldShoot() {
        return false;
    }

    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void fixPosAfterShooting() {
        //Do nothing
    }

    @Override
    public void moveAcrossField(double power) {
        goDiagonalBackwardLeft(power);
    }

    @Override
    public void moveAcrossFieldDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalBackwardLeftDistance(power, centimeters);
    }

    @Override
    public void moveAlongBeaconWall(double power) {
        goBackward(power);
    }

    @Override
    public void moveAlongBeaconWallDistance(double power, double centimeters) throws InterruptedException {
        goBackwardDistance(power, centimeters);
    }

    @Override
    public void fixPosForFindingTape() throws InterruptedException {
        goBackwardDistance(DEFAULT_FORWARD_SPEED, 15);
    }

    @Override
    public void findTapeInward() throws InterruptedException {
        findTapeLeft();
    }

    @Override
    public void findTapeOutward() throws InterruptedException {
        findTapeRight();
    }

    @Override
    public void moveCorrectButtonFlap() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        if (leftBlue > rightBlue && leftRed < rightRed) {
            leftFlapServo.setPosition(leftFlapEndPos);
            rightFlapServo.setPosition(rightFlapInitPos);
        } else if (rightBlue > leftBlue && rightRed < leftRed) {
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        }
    }

    @Override
    public void setTeleOpAngle() {
        CompetitionTeleOp.currentAngle = 270;
    }
}
