package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.TypeConversion;

public abstract class AutonomousOpMode extends BaseOpMode {

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void spinRight(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void spinLeft(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goForward(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
    }

    public void goBackward(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
    }

    public void goLeft(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goRight(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalForwardRight(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(0);
    }

    public void goDiagonalForwardLeft(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-power);
    }

    public void goDiagonalBackwardRight(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalBackwardLeft(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(0);
    }

    public void moveBackWheelsLeft(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveBackWheelsRight(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveFrontWheelsLeft(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void moveFrontWheelsRight(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void moveLeftWheelsForward(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(0);
    }

    public void moveLeftWheelsBackward(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(0);
    }

    public void moveRightWheelsForward(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-power);
    }

    public void moveRightWheelsBackward(double power) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(power);
    }

    public void spinRightDegrees(double power, double degrees) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        spinRight(power);
        double totalEncoderTicks = degrees * TICKS_PER_DEGREE;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(power) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        stopRobot();
    }

    public void spinLeftDegrees(double power, double degrees) throws InterruptedException {
        spinRightDegrees(-power, degrees);
    }

    public void goForwardDistance(double power, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        goForward(power);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_FORWARD;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(power) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        stopRobot();
    }

    public void goBackwardDistance(double power, double centimeters) throws InterruptedException {
        goForwardDistance(-power, centimeters);
    }

    public void goRightDistance(double power, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        goRight(power);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_SIDEWAYS;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(power) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            logData("backLeft", String.valueOf(backLeftMotor.getCurrentPosition()));
            logData("backRight", String.valueOf(backRightMotor.getCurrentPosition()));
            logData("frontLeft", String.valueOf(frontLeftMotor.getCurrentPosition()));
            logData("frontRight", String.valueOf(frontRightMotor.getCurrentPosition()));
            updateTelemetry();
        }
        stopRobot();
    }

    public void goLeftDistance(double power, double centimeters) throws InterruptedException {
        goRightDistance(-power, centimeters);
    }

    private void goDiagonalDistance(double power, double centimeters, int forward, int sideways) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        if (forward == 1 && sideways == 1) {
            goDiagonalForwardRight(power);
        } else if (forward == 1 && sideways == -1) {
            goDiagonalForwardLeft(power);
        } else if (forward == -1 && sideways == 1) {
            goDiagonalBackwardRight(power);
        } else if (forward == -1 && sideways == -1) {
            goDiagonalBackwardLeft(power);
        }
        double totalEncoderTicks = centimeters * TICKS_PER_CM_DIAGONAL;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            if ((forward == 1 && sideways == 1) || (forward == -1 && sideways == -1)) {
                fixRpm(Math.abs(power) * wheelMaxRpm, wheelEncoderPpr, backRightMotor, frontLeftMotor);
            } else if ((forward == 1 && sideways == -1) || (forward == -1 && sideways == 1)) {
                fixRpm(Math.abs(power) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, frontRightMotor);
            }
        }
        stopRobot();
    }

    public void goDiagonalForwardRightDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalDistance(power, centimeters, 1, 1);
    }

    public void goDiagonalForwardLeftDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalDistance(power, centimeters, 1, -1);
    }

    public void goDiagonalBackwardRightDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalDistance(power, centimeters, -1, 1);
    }

    public void goDiagonalBackwardLeftDistance(double power, double centimeters) throws InterruptedException {
        goDiagonalDistance(power, centimeters, -1, -1);
    }

    public int getFurthestEncoder() {
        return Math.max(Math.max(Math.abs(backLeftMotor.getCurrentPosition()), Math.abs(backRightMotor.getCurrentPosition())), Math.max(Math.abs(frontLeftMotor.getCurrentPosition()), Math.abs(frontRightMotor.getCurrentPosition())));
    }

    public int getRightRangeDistance() {
        return rightRangeSensor.rawUltrasonic();
    }

    public int getLeftRangeDistance() {
        return leftRangeSensor.rawUltrasonic();
    }

    public int[] getAverageColor(ColorSensor... colorSensors) throws InterruptedException {
        int n = colorSensors.length;
        double[] averageAlpha = new double[n];
        double[] averageRed = new double[n];
        double[] averageGreen = new double[n];
        double[] averageBlue = new double[n];
        int numReads = 10;
        for (int i = 0; i < numReads; i++) {
            for (int j = 0; j < n; j++) {
                int color = colorSensors[j].argb();
                averageAlpha[j] += Color.alpha(color);
                averageRed[j] += Color.red(color);
                averageGreen[j] += Color.green(color);
                averageBlue[j] += Color.blue(color);
            }
            sleep(34);//"internal sampling rate" is 30 times per second, according to Modern Robotics  (1/30) * 1000 = 33.3
        }
        int[] averageColor = new int[n];
        for (int j = 0; j < n; j++) {
            averageAlpha[j] /= numReads;
            averageRed[j] /= numReads;
            averageGreen[j] /= numReads;
            averageBlue[j] /= numReads;
            averageColor[j] = Color.argb((int) Math.round(averageAlpha[j]), (int) Math.round(averageRed[j]), (int) Math.round(averageGreen[j]), (int) Math.round(averageBlue[j]));
        }
        return averageColor;
    }
}