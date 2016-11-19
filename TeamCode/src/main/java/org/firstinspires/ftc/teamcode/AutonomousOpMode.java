package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class AutonomousOpMode extends BaseOpMode {

    public void stopRobot() {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

    public void goForward(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(-power);
    }

    public void goBackward(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
    }

    public void goLeft(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
    }

    public void goRight(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void goDiagonalForwardRight(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(0);
    }

    public void goDiagonalForwardLeft(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(-power);
    }

    public void goDiagonalBackwardRight(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(power);
    }

    public void goDiagonalBackwardLeft(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(0);
    }

    public void moveBackWheelsLeft(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

    public void moveBackWheelsRight(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

    public void moveFrontWheelsLeft(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
    }

    public void moveFrontWheelsRight(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void moveLeftWheelsForward(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(0);
    }

    public void moveLeftWheelsBackward(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(0);
    }

    public void moveRightWheelsForward(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(-power);
    }

    public void moveRightWheelsBackward(double power) {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(power);
    }

    public void goForwardDistance(double power, double centimeters) {
        resetEncoders();
        double totalEncoderTicks = centimeters * TICKS_PER_CM_FORWARD;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            goForward(power);
        }
        stopRobot();
    }

    public void goBackwardDistance(double power, double centimeters){
        goForwardDistance(-power, centimeters);
    }

    public void goRightDistance(double power, double centimeters) {
        resetEncoders();
        double totalEncoderTicks = centimeters * TICKS_PER_CM_SIDEWAYS;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            goRight(power);
        }
        stopRobot();
    }

    public void goLeftDistance(double power, double centimeters) {
        goRightDistance(-power, centimeters);
    }

    private void goDiagonalDistance(double power, double centimeters, int forward, int sideways) {
        resetEncoders();
        double totalEncoderTicks = centimeters * TICKS_PER_CM_DIAGONAL;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            if (forward == 1 && sideways == 1) {
                goDiagonalForwardRight(power);
            } else if (forward == 1 && sideways == -1) {
                goDiagonalForwardLeft(power);
            } else if (forward == -1 && sideways == 1) {
                goDiagonalBackwardRight(power);
            } else if (forward == -1 && sideways == -1) {
                goDiagonalBackwardLeft(power);
            }
        }
        stopRobot();
    }

    public void goDiagonalForwardRightDistance(double power, double centimeters) {
        goDiagonalDistance(power, centimeters, 1, 1);
    }

    public void goDiagonalForwardLeftDistance(double power, double centimeters) {
        goDiagonalDistance(power, centimeters, 1, -1);
    }

    public void goDiagonalBackwardRightDistance(double power, double centimeters) {
        goDiagonalDistance(power, centimeters, -1, 1);
    }

    public void goDiagonalBackwardLeftDistance(double power, double centimeters) {
        goDiagonalDistance(power, centimeters, -1, -1);
    }

    public int getFurthestEncoder() {
        return Math.max(Math.max(Math.abs(motorBackLeft.getCurrentPosition()), Math.abs(motorBackRight.getCurrentPosition())), Math.max(Math.abs(motorFrontLeft.getCurrentPosition()), Math.abs(motorFrontRight.getCurrentPosition())));
    }

    public int getRightRangeDistance() {
        return rightRangeReader.read(0x04, 1)[0];
    }

    public int getLeftRangeDistance() {
        return leftRangeReader.read(0x04, 1)[0];
    }

    public double getAverageRed(ColorSensor colorSensor) throws InterruptedException {
        double averageRed = 0;
        int numReads = 100;
        for (int i = 0; i < numReads; i++) {
            averageRed += colorSensor.red();
            sleep(1);
        }
        averageRed /= numReads;
        return averageRed;
    }

    public double getAverageBlue(ColorSensor colorSensor) throws InterruptedException {
        double averageBlue = 0;
        int numReads = 100;
        for (int i = 0; i < numReads; i++) {
            averageBlue += colorSensor.blue();
            sleep(1);
        }
        averageBlue /= numReads;
        return averageBlue;
    }
}