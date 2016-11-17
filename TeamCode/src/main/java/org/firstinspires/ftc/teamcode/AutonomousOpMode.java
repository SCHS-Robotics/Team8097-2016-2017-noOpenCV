package org.firstinspires.ftc.teamcode;

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
        motorBackLeft.setPower(0);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(0);
    }

    public void goDiagonalBackwardLeft(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(power);
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
}