package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public abstract class AutonomousOpMode extends BaseOpMode {

    public void moveBackWheelsLeft(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveBackWheelsRight(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveFrontWheelsLeft(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
    }

    public void moveFrontWheelsRight(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    public void moveLeftWheelsForward(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(0);
    }

    public void moveLeftWheelsBackward(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(0);
    }

    public void moveRightWheelsForward(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-speed);
    }

    public void moveRightWheelsBackward(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(speed);
    }

    public void spinRightDegrees(double speed, double degrees) throws InterruptedException {
        resetWheelEncoders();
        spinRight(speed);
        double totalEncoderTicks = degrees * TICKS_PER_DEGREE;
        waitForEncoders(totalEncoderTicks);
        stopRobot();
    }

    public void spinLeftDegrees(double speed, double degrees) throws InterruptedException {
        spinRightDegrees(-speed, degrees);
    }

    public void goForwardDistance(double speed, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        goForward(speed);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_FORWARD;
        waitForEncoders(totalEncoderTicks);
        stopRobot();
    }

    private void waitForEncoders(double encoderTicks) throws InterruptedException {
        while (getFurthestEncoder() < encoderTicks && opModeIsActive()) {
            sleep(1);
        }
    }

    public void goBackwardDistance(double speed, double centimeters) throws InterruptedException {
        goForwardDistance(-speed, centimeters);
    }

    public void goRightDistance(double speed, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        goRight(speed);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_SIDEWAYS;
        waitForEncoders(totalEncoderTicks);
        stopRobot();
    }

    public void goLeftDistance(double speed, double centimeters) throws InterruptedException {
        goRightDistance(-speed, centimeters);
    }

    private void goDiagonalDistance(double speed, double centimeters, int forward, int sideways) throws InterruptedException {
        resetWheelEncoders();
        if (forward == 1 && sideways == 1) {
            goDiagonalForwardRight(speed);
        } else if (forward == 1 && sideways == -1) {
            goDiagonalForwardLeft(speed);
        } else if (forward == -1 && sideways == 1) {
            goDiagonalBackwardRight(speed);
        } else if (forward == -1 && sideways == -1) {
            goDiagonalBackwardLeft(speed);
        }
        double totalEncoderTicks = centimeters * TICKS_PER_CM_DIAGONAL;
        waitForEncoders(totalEncoderTicks);
        stopRobot();
    }

    public void goDiagonalForwardRightDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalDistance(speed, centimeters, 1, 1);
    }

    public void goDiagonalForwardLeftDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalDistance(speed, centimeters, 1, -1);
    }

    public void goDiagonalBackwardRightDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalDistance(speed, centimeters, -1, 1);
    }

    public void goDiagonalBackwardLeftDistance(double speed, double centimeters) throws InterruptedException {
        goDiagonalDistance(speed, centimeters, -1, -1);
    }

    public int getFurthestEncoder() {
        return Math.max(Math.max(Math.abs(backLeftMotor.getCurrentPosition()), Math.abs(backRightMotor.getCurrentPosition())), Math.max(Math.abs(frontLeftMotor.getCurrentPosition()), Math.abs(frontRightMotor.getCurrentPosition())));
    }

//    public int getRightRangeDistance() {
//        return rightRangeSensor.rawUltrasonic();
//    }

    public int getRangeDistance() {
        return rangeSensor.rawUltrasonic();
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

    public double determineAngleOffset() throws InterruptedException {
        double angleWindow = 60;
        double startPos = rangeServoInitPos - ((angleWindow / 2) * SERVO_POS_PER_DEGREE);
        double angleIncrement = 2;
        int numReads = (int) Math.round(angleWindow / angleIncrement);
        int minDistance = 255;
        double firstMinPos = 0;
        double lastMinPos = 1;
        for (int i = 0; i < numReads; i++) {
            double pos = startPos + (angleIncrement * SERVO_POS_PER_DEGREE * i);
            rangeServo.setPosition(pos);
            sleep(100);
            int ultrasonic = rangeSensor.rawUltrasonic();
            logData("distance", ultrasonic);
            updateTelemetry();
            if (ultrasonic < minDistance) {
                minDistance = ultrasonic;
                firstMinPos = pos;
            } else if (ultrasonic == minDistance) {
                lastMinPos = pos;
            }
        }
        rangeServo.setPosition(rangeServoInitPos);
        return -(((lastMinPos + firstMinPos) / 2) - 0.5) / SERVO_POS_PER_DEGREE;
    }
}