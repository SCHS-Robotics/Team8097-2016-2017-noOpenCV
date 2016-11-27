package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public abstract class AutonomousOpMode extends BaseOpMode {

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void spinRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void spinLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goForward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
    }

    public void goBackward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
    }

    public void goLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalForwardRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(0);
    }

    public void goDiagonalForwardLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-power);
    }

    public void goDiagonalBackwardRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalBackwardLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(0);
    }

    public void moveBackWheelsLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveBackWheelsRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void moveFrontWheelsLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void moveFrontWheelsRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void moveLeftWheelsForward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(0);
    }

    public void moveLeftWheelsBackward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(0);
    }

    public void moveRightWheelsForward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-power);
    }

    public void moveRightWheelsBackward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(power);
    }

    public void spinRightDegrees(double percentSpeed, double degrees) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        spinRight(percentSpeed);
        double totalEncoderTicks = degrees * TICKS_PER_DEGREE;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(percentSpeed) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        stopRobot();
    }

    public void spinLeftDegrees(double percentSpeed, double degrees) throws InterruptedException {
        spinRightDegrees(-percentSpeed, degrees);
    }

    public void goForwardDistance(double percentSpeed, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        goForward(percentSpeed);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_FORWARD;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(percentSpeed) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
        }
        stopRobot();
    }

    public void goBackwardDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goForwardDistance(-percentSpeed, centimeters);
    }

    public void goRightDistance(double percentSpeed, double centimeters) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        goRight(percentSpeed);
        double totalEncoderTicks = centimeters * TICKS_PER_CM_SIDEWAYS;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            fixRpm(Math.abs(percentSpeed) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            logData("backLeft", String.valueOf(backLeftMotor.getCurrentPosition()));
            logData("backRight", String.valueOf(backRightMotor.getCurrentPosition()));
            logData("frontLeft", String.valueOf(frontLeftMotor.getCurrentPosition()));
            logData("frontRight", String.valueOf(frontRightMotor.getCurrentPosition()));
            updateTelemetry();
        }
        stopRobot();
    }

    public void goLeftDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goRightDistance(-percentSpeed, centimeters);
    }

    private void goDiagonalDistance(double percentSpeed, double centimeters, int forward, int sideways) throws InterruptedException {
        resetWheelEncoders();
        fixRpmTimer.reset();
        if (forward == 1 && sideways == 1) {
            goDiagonalForwardRight(percentSpeed);
        } else if (forward == 1 && sideways == -1) {
            goDiagonalForwardLeft(percentSpeed);
        } else if (forward == -1 && sideways == 1) {
            goDiagonalBackwardRight(percentSpeed);
        } else if (forward == -1 && sideways == -1) {
            goDiagonalBackwardLeft(percentSpeed);
        }
        double totalEncoderTicks = centimeters * TICKS_PER_CM_DIAGONAL;
        while (getFurthestEncoder() < totalEncoderTicks && opModeIsActive()) {
            if ((forward == 1 && sideways == 1) || (forward == -1 && sideways == -1)) {
                fixRpm(Math.abs(percentSpeed) * wheelMaxRpm, wheelEncoderPpr, backRightMotor, frontLeftMotor);
            } else if ((forward == 1 && sideways == -1) || (forward == -1 && sideways == 1)) {
                fixRpm(Math.abs(percentSpeed) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, frontRightMotor);
            }
        }
        stopRobot();
    }

    public void goDiagonalForwardRightDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goDiagonalDistance(percentSpeed, centimeters, 1, 1);
    }

    public void goDiagonalForwardLeftDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goDiagonalDistance(percentSpeed, centimeters, 1, -1);
    }

    public void goDiagonalBackwardRightDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goDiagonalDistance(percentSpeed, centimeters, -1, 1);
    }

    public void goDiagonalBackwardLeftDistance(double percentSpeed, double centimeters) throws InterruptedException {
        goDiagonalDistance(percentSpeed, centimeters, -1, -1);
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
        return (((lastMinPos + firstMinPos) / 2) - 0.5) / SERVO_POS_PER_DEGREE;//TODO + or - ???
    }
}