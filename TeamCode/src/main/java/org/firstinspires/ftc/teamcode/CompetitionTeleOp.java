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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

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
    private ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final int waitTime = 20;

    private int numAverage = 10;//10 averages waiting 20 milliseconds each takes 200 milliseconds.
    private Double[] backLeftMotorPower = new Double[numAverage];
    private Double[] backRightMotorPower = new Double[numAverage];
    private Double[] frontLeftMotorPower = new Double[numAverage];
    private Double[] frontRightMotorPower = new Double[numAverage];
    private double backLeftPercentError = 0;
    private double backRightPercentError = 0;
    private double frontLeftPercentError = 0;
    private double frontRightPercentError = 0;

    public static double currentAngle = 90;//initialized in autonomous based on which one is run.
    boolean spun = false;

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

        resetPowers();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        pushButtonTime.reset();
        waitTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());
            updateTelemetry();

            //Movement
            if (waitTimer.time() >= waitTime) {
                waitTimer.reset();
                if (backLeftMotorPower[numAverage - 1] != null) {
                    if (enoughData(backLeftMotorPower))
                        backLeftPercentError = (getPercentError(averagePower(backLeftMotorPower) * wheelMaxRpm, wheelEncoderPpr, backLeftMotor, waitTime * numAverage) + 1) * (backLeftPercentError + 1) - 1;
                    if (enoughData(backRightMotorPower))
                        backRightPercentError = (getPercentError(averagePower(backRightMotorPower) * wheelMaxRpm, wheelEncoderPpr, backRightMotor, waitTime * numAverage) + 1) * (backRightPercentError + 1) - 1;
                    if (enoughData(frontLeftMotorPower))
                        frontLeftPercentError = (getPercentError(averagePower(frontLeftMotorPower) * wheelMaxRpm, wheelEncoderPpr, frontLeftMotor, waitTime * numAverage) + 1) * (frontLeftPercentError + 1) - 1;
                    if (enoughData(frontRightMotorPower))
                        frontRightPercentError = (getPercentError(averagePower(frontRightMotorPower) * wheelMaxRpm, wheelEncoderPpr, frontRightMotor, waitTime * numAverage) + 1) * (frontRightPercentError + 1) - 1;
                    resetPowers();
                }
                if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                    if (!spun) {
                        resetWheelEncoders();
                        waitTimer.reset();
                        resetPowers();
                    }
                    if (gamepad1.right_trigger > 0) {
                        spinRight(gamepad1.right_trigger / 2.0, backLeftPercentError, backRightPercentError, frontLeftPercentError, frontRightPercentError);
                    } else if (gamepad1.left_trigger > 0) {
                        spinLeft(gamepad1.left_trigger / 2.0, backLeftPercentError, backRightPercentError, frontLeftPercentError, frontRightPercentError);
                    }
                    spun = true;
                } else {
                    if (spun) {
                        double averageTicks = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()) / 4.0;
                        currentAngle += -averageTicks / TICKS_PER_DEGREE;
                        resetPowers();
                    }
                    double joystickInputX = gamepad1.left_stick_x;
                    double joystickInputY = -gamepad1.left_stick_y;
                    double magnitude = Math.sqrt(Math.pow(joystickInputX, 2) + Math.pow(joystickInputY, 2));
                    double angle = Math.toDegrees(Math.atan2(joystickInputY, joystickInputX));
                    angle -= (currentAngle - 90);
                    goDirectionPolar(magnitude, angle, backLeftPercentError, backRightPercentError, frontLeftPercentError, frontRightPercentError);
                    spun = false;
                }
                updatePowers();
            }

            if (gamepad1.guide) {
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

    public void goDirectionPolar(double magnitude, double angle) {
        double x = magnitude * Math.cos(Math.toRadians(angle));
        double y = magnitude * Math.sin(Math.toRadians(angle));
        goDirection(x, y);
    }

    public void goDirectionPolar(double magnitude, double angle, double backLeftPercentError, double backRightPercentError, double frontLeftPercentError, double frontRightPercentError) {
        double x = magnitude * Math.cos(Math.toRadians(angle));
        double y = magnitude * Math.sin(Math.toRadians(angle));
        goDirection(x, y, backLeftPercentError, backRightPercentError, frontLeftPercentError, frontRightPercentError);
    }

    public void goDirection(double x, double y) {
        double sqrt2 = Math.sqrt(2);
        backLeftMotor.setPower((y - x) / sqrt2);
        backRightMotor.setPower((-y - x) / sqrt2);
        frontLeftMotor.setPower((y + x) / sqrt2);
        frontRightMotor.setPower((-y + x) / sqrt2);
    }

    public void goDirection(double x, double y, double backLeftPercentError, double backRightPercentError, double frontLeftPercentError, double frontRightPercentError) {
        double sqrt2 = Math.sqrt(2);
        backLeftMotor.setPower(((y - x) / sqrt2) / (backLeftPercentError + 1));
        backRightMotor.setPower(((-y - x) / sqrt2) / (backRightPercentError + 1));
        frontLeftMotor.setPower(((y + x) / sqrt2) / (frontLeftPercentError + 1));
        frontRightMotor.setPower(((-y + x) / sqrt2) / (frontRightPercentError + 1));
    }

    public void spinRight(double power, double backLeftPercentError, double backRightPercentError, double frontLeftPercentError, double frontRightPercentError) {
        backLeftMotor.setPower(power / (backLeftPercentError + 1));
        backRightMotor.setPower(power / (backRightPercentError + 1));
        frontLeftMotor.setPower(power / (frontLeftPercentError + 1));
        frontRightMotor.setPower(power / (frontRightPercentError + 1));
    }

    public void spinLeft(double power, double backLeftPercentError, double backRightPercentError, double frontLeftPercentError, double frontRightPercentError) {
        backLeftMotor.setPower(-power / (backLeftPercentError + 1));
        backRightMotor.setPower(-power / (backRightPercentError + 1));
        frontLeftMotor.setPower(-power / (frontLeftPercentError + 1));
        frontRightMotor.setPower(-power / (frontRightPercentError + 1));
    }

    private void resetPowers() {
        Arrays.fill(backLeftMotorPower, null);
        Arrays.fill(backRightMotorPower, null);
        Arrays.fill(frontLeftMotorPower, null);
        Arrays.fill(frontRightMotorPower, null);
        encoderStartPos.put(backLeftMotor, backLeftMotor.getCurrentPosition());
        encoderStartPos.put(backRightMotor, backRightMotor.getCurrentPosition());
        encoderStartPos.put(frontLeftMotor, frontLeftMotor.getCurrentPosition());
        encoderStartPos.put(frontRightMotor, frontRightMotor.getCurrentPosition());
    }

    private void updatePowers() {
        for (int i = 0; i < numAverage - 1; i++) {
            backLeftMotorPower[i + 1] = backLeftMotorPower[i];
            backRightMotorPower[i + 1] = backRightMotorPower[i];
            frontLeftMotorPower[i + 1] = frontLeftMotorPower[i];
            frontRightMotorPower[i + 1] = frontRightMotorPower[i];
        }
        backLeftMotorPower[0] = backLeftMotor.getPower() * (backLeftPercentError + 1);
        backRightMotorPower[0] = backRightMotor.getPower() * (backRightPercentError + 1);
        frontLeftMotorPower[0] = frontLeftMotor.getPower() * (frontLeftPercentError + 1);
        frontRightMotorPower[0] = frontRightMotor.getPower() * (frontRightPercentError + 1);
    }

    private double averagePower(Double[] power) {
        double average = 0;
        for (int i = 0; i < numAverage; i++) {
            average += power[i];
        }
        average /= numAverage;
        return average;
    }

    private boolean enoughData(Double[] power) {
        for (int i = 0; i < numAverage; i++) {
            if (Math.abs(power[i]) < 0.2) {
                return false;
            }
        }
        return true;
    }
}
