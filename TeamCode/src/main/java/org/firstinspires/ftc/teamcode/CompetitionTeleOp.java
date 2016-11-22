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
    private ElapsedTime pushButtonTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        pushButtonTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //Movement
            if (gamepad1.right_trigger > 0) {
                spinRight(gamepad1.right_trigger / 2.0);
            } else if (gamepad1.left_trigger > 0) {
                spinLeft(gamepad1.left_trigger / 2.0);
            } else {
                double joystickInputY = -gamepad1.left_stick_y;
                double joystickInputX = gamepad1.left_stick_x;
                goDirection(joystickInputX, joystickInputY);
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
            if (pushButtonTime.time() >= 0.5) {
                rightFlapServo.setPosition(rightFlapInitPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    protected void goDirection(double x, double y) {
        backLeftMotor.setPower((y - x) / 1.0);
        backRightMotor.setPower((-y - x) / 1.0);
        frontLeftMotor.setPower((y + x) / 1.0);
        frontRightMotor.setPower((-y + x) / 1.0);
    }
}
