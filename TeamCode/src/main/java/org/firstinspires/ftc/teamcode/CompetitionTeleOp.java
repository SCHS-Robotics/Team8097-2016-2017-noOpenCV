package org.firstinspires.ftc.teamcode;/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.HashMap;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p/>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p/>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOp", group = "OpMode")
// @Autonomous(...) is the other common choice
public class CompetitionTeleOp extends BaseOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pushButtonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    final int waitTime = 100;

    public static double currentAngle = 90;//initialized in autonomous based on which one is run.
    boolean spun = false;
    public final static int NONE = -1;
    public final static int SPIN_RIGHT = 1;
    public final static int SPIN_LEFT = 2;
    public final static int RIGHT = 0;
    public final static int FORWARD_RIGHT = 45;
    public final static int FORWARD = 90;
    public final static int FORWARD_LEFT = 135;
    public final static int LEFT = 180;
    public final static int BACKWARD_LEFT = 225;
    public final static int BACKWARD = 270;
    public final static int BACKWARD_RIGHT = 315;
    int movement = NONE;

    public final static double MIN_SPEED = 0.25;

    HashMap<DcMotor, Double> percentErrors;
    double prevRpm;


    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);

        resetPercentErrors();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        pushButtonTime.reset();
        fixRpmTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());
            updateTelemetry();

            //Movement
            if (fixRpmTimer.time() >= waitTime) {
                resetWheelEncoderStartPos();
                fixRpmTimer.reset();
                if (gamepad1.right_trigger > MIN_SPEED || gamepad1.left_trigger > MIN_SPEED) {
                    if (!spun) {
                        resetWheelEncoders();
                        fixRpmTimer.reset();//resetting the encoders takes time
                    }
                    if (gamepad1.right_trigger > MIN_SPEED) {
                        if (movement != SPIN_RIGHT) {
                            prevRpm = 0;
                            resetPercentErrors();
                            spinRight(gamepad1.right_trigger);
                        } else {
                            fixRpmTeleOp(Math.abs(gamepad1.right_trigger) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
                        }
                        movement = SPIN_RIGHT;
                    } else if (gamepad1.left_trigger > MIN_SPEED) {
                        if (movement != SPIN_LEFT) {
                            prevRpm = 0;
                            resetPercentErrors();
                            spinLeft(gamepad1.left_trigger);
                        } else {
                            fixRpmTeleOp(Math.abs(gamepad1.left_trigger) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
                        }
                        movement = SPIN_LEFT;
                    }
                    spun = true;
                } else {
                    if (spun) {
                        double averageTicks = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()) / 4.0;
                        currentAngle += -averageTicks / TICKS_PER_DEGREE;
                    }
                    double joystickInputX = gamepad1.left_stick_x;
                    double joystickInputY = -gamepad1.left_stick_y;
                    double magnitude = Math.sqrt(Math.pow(joystickInputX, 2) + Math.pow(joystickInputY, 2));
                    double angle = Math.toDegrees(Math.atan2(joystickInputY, joystickInputX));
                    angle -= (currentAngle - 90);
                    if (magnitude > MIN_SPEED) {
                        goDirection(magnitude, angle);
                    } else {
                        stopRobot();
                        movement = NONE;
                    }
                    spun = false;
                }
            }

            if (gamepad1.right_stick_button) {
                currentAngle = 90;
                logData("Angle calibrated!", "Angle calibrated!");
            }

            //Button Pushers
            if (gamepad1.left_bumper) {
                pushButtonTime.reset();
                leftFlapServo.setPosition(leftFlapEndPos);
                rightFlapServo.setPosition(rightFlapInitPos);
            } else if (gamepad1.right_bumper) {
                pushButtonTime.reset();
                rightFlapServo.setPosition(rightFlapEndPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }
            if (pushButtonTime.time() >= 500) {
                rightFlapServo.setPosition(rightFlapInitPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public double goDirection(double magnitude, double angle) throws InterruptedException {
        if (angleIsNearAngle(angle, RIGHT)) {
            if (movement != RIGHT) {
                prevRpm = 0;
                resetPercentErrors();
                goRight(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            }
            movement = RIGHT;
        } else if (angleIsNearAngle(angle, FORWARD_RIGHT)) {
            if (movement != FORWARD_RIGHT) {
                prevRpm = 0;
                resetPercentErrors();
                goDiagonalForwardRight(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backRightMotor, frontLeftMotor);
            }
            movement = FORWARD_RIGHT;
        } else if (angleIsNearAngle(angle, FORWARD)) {
            if (movement != FORWARD) {
                prevRpm = 0;
                resetPercentErrors();
                goForward(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            }
            movement = FORWARD;
        } else if (angleIsNearAngle(angle, FORWARD_LEFT)) {
            if (movement != FORWARD_LEFT) {
                prevRpm = 0;
                resetPercentErrors();
                goDiagonalForwardLeft(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, frontRightMotor);
            }
            movement = FORWARD_LEFT;
        } else if (angleIsNearAngle(angle, LEFT)) {
            if (movement != LEFT) {
                prevRpm = 0;
                resetPercentErrors();
                goLeft(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            }
            movement = LEFT;
        } else if (angleIsNearAngle(angle, BACKWARD_LEFT)) {
            if (movement != BACKWARD_LEFT) {
                prevRpm = 0;
                resetPercentErrors();
                goDiagonalBackwardLeft(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backRightMotor, frontLeftMotor);
            }
            movement = BACKWARD_LEFT;
        } else if (angleIsNearAngle(angle, BACKWARD)) {
            if (movement != BACKWARD) {
                prevRpm = 0;
                resetPercentErrors();
                goBackward(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
            }
            movement = BACKWARD;
        } else if (angleIsNearAngle(angle, BACKWARD_RIGHT)) {
            if (movement != BACKWARD_RIGHT) {
                prevRpm = 0;
                resetPercentErrors();
                goDiagonalBackwardRight(magnitude);
            } else {
                fixRpmTeleOp(Math.abs(magnitude) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, frontRightMotor);
            }
            movement = BACKWARD_RIGHT;
        } else {
            stopRobot();
            movement = NONE;
            logData("ERROR!!!", "ERROR!!!");
        }

        return angle;
    }

    public boolean angleIsNearAngle(double angle1, double angle2) {
        while (angle1 >= 360) {
            angle1 -= 360;
        }
        while (angle1 < 0) {
            angle1 += 360;
        }
        while (angle2 >= 360) {
            angle2 -= 360;
        }
        while (angle2 < 0) {
            angle2 += 360;
        }
        double diff = Math.abs(angle2 - angle1);
        return diff < 45.0 / 2 || diff > 360 - 45.0 / 2;
    }

    public void fixRpmTeleOp(double rpm, int encoderPpr, DcMotor... motors) throws InterruptedException {
        if (prevRpm == 0) {
            prevRpm = getRpmEstimate(Math.abs(motors[0].getPower()));
        }
        for (int i = 0; i < motors.length; i++) {
            DcMotor motor = motors[i];
            double currentRpm = getCurrentRpm(encoderPpr, motor, waitTime);
            percentErrors.put(motor, (percentErrors.get(motor) + 1) * (getPercentError(prevRpm, currentRpm) + 1) - 1);
            encoderStartPos.put(motor, Math.abs(motor.getCurrentPosition()));
            double modifiedRpm = rpm / (percentErrors.get(motor) + 1);
            double newPower = Math.signum(motor.getPower()) * getPowerEstimate(modifiedRpm);
            motor.setPower(newPower);
        }
        prevRpm = rpm;
    }

    private void resetPercentErrors() {
        percentErrors.put(backLeftMotor, 0.0);
        percentErrors.put(backRightMotor, 0.0);
        percentErrors.put(frontLeftMotor, 0.0);
        percentErrors.put(frontRightMotor, 0.0);
    }
}
