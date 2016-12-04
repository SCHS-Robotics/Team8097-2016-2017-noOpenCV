package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Autonomous", group = "OpMode")
public class BlueAutonomousOpMode extends CompetitionAutonomousOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        CompetitionTeleOp.currentAngle = 270;
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
}
