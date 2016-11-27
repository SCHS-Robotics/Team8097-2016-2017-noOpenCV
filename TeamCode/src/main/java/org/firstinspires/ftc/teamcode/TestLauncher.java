/*
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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p/>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p/>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Calibrate Launcher", group = "Util")
public class TestLauncher extends BaseOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launcherTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double pos = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(0);

        waitForStart();
        runtime.reset();
        launcherTime.reset();
        resetEncoders(rightLaunchMotor, leftLaunchMotor);
        launchMotorsRpm(1500);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());

            if (gamepad1.y) {
                if (pos + 0.002 <= 1)
                    pos += 0.002;
            } else if (gamepad1.a) {
                if (pos - 0.002 >= 0)
                    pos -= 0.002;
            }
            logData("position", pos);
            updateTelemetry();
            launcherServo.setPosition(pos);
            sleep(10);


            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }

    public void launchMotorsRpm(double rpm) throws InterruptedException {
        final int waitTime = 250;
        double leftPercentError = 0;
        double rightPercentError = 0;
//        resetEncoders(leftLaunchMotor, rightLaunchMotor);
        int leftStartPos = 0;
        int rightStartPos = 0;
        leftLaunchMotor.setPower(-0.5);
        rightLaunchMotor.setPower(0.5);
        sleep(waitTime);
        while ((Math.abs(1 - leftPercentError) > 0.015 || (Math.abs(1 - rightPercentError) > 0.015)) && opModeIsActive()) {
            launcherTime.reset();

            double currentLeftRpm = ((Math.abs(leftLaunchMotor.getCurrentPosition()) - leftStartPos) / 112.0) / (waitTime / 60000.0);
            leftPercentError = currentLeftRpm / rpm;
            logData("left encoder", Math.abs(leftLaunchMotor.getCurrentPosition()) - leftStartPos);
            logData("leftPercentError", leftPercentError);
            leftStartPos = Math.abs(leftLaunchMotor.getCurrentPosition());
            leftLaunchMotor.setPower(-(Math.abs(leftLaunchMotor.getPower()) / leftPercentError));
            logData("left power", leftLaunchMotor.getPower());

            double currentRightRpm = ((Math.abs(rightLaunchMotor.getCurrentPosition()) - rightStartPos) / 112.0) / (waitTime / 60000.0);
            rightPercentError = currentRightRpm / rpm;
            logData("right encoder", Math.abs(rightLaunchMotor.getCurrentPosition()) - rightStartPos);
            logData("rightPercentError", rightPercentError);
            rightStartPos = Math.abs(rightLaunchMotor.getCurrentPosition());
            rightLaunchMotor.setPower((Math.abs(rightLaunchMotor.getPower()) / rightPercentError));
            logData("right power", rightLaunchMotor.getPower());

            updateTelemetry();

            if (waitTime - launcherTime.time() > 0) {
                sleep((int) (waitTime - launcherTime.time()));
            }
        }
    }
}
