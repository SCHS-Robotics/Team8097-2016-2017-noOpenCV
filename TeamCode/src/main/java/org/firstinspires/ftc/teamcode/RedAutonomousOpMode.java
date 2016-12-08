package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Autonomous", group = "OpMode")
public class RedAutonomousOpMode extends CompetitionAutonomousOpMode {

    @Override
    public boolean shouldShoot() {
        return false;
    }

    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void fixPosAfterShooting() throws InterruptedException {
        spinRightDegrees(DEFAULT_SPIN_SPEED, 180);
    }

    @Override
    public void moveAcrossField(double power) {
        goDiagonalForwardLeft(power);
    }

    @Override
    public void moveAcrossFieldDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalForwardLeftDistance(power, centimeters);
    }

    @Override
    public void moveAlongBeaconWall(double power) {
        goForward(power);
    }

    @Override
    public void moveAlongBeaconWallDistance(double power, double centimeters) throws InterruptedException {
        goForwardDistance(power, centimeters);
    }

    @Override
    public void findTapeInward() throws InterruptedException {
        findTapeRight();
    }

    @Override
    public void findTapeOutward() throws InterruptedException {
        findTapeLeft();
    }

    @Override
    public void moveCorrectButtonFlap() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        if (leftRed > rightRed && leftBlue < rightBlue) {
            leftFlapServo.setPosition(leftFlapEndPos);
            rightFlapServo.setPosition(rightFlapInitPos);
        } else if (rightRed > leftRed && rightBlue < leftBlue) {
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        } else { //testing without working beacon only
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        }
    }

    @Override
    public void setTeleOpAngle() {
        CompetitionTeleOp.currentAngle = 90;
    }
}
