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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Test Encoders", group = "Test")
public class TestEncoders extends AutonomousOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double power = 0;
    int waitTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

//        allInit();
        backLeftMotor = hardwareMap.dcMotor.get("motor");

        waitForStart();
        runtime.reset();
//        resetWheelEncoders();
        resetEncoders(backLeftMotor);
        fixRpmTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            logData("Status", "Run Time: " + runtime.toString());
//            fixRpm(wheelMaxRpm, wheelEncoderPpr, backLeftMotor);

//            goForward(DEFAULT_FORWARD_SPEED);
//            logData("backLeft", backLeftMotor.getCurrentPosition());
//            logData("backRight", backRightMotor.getCurrentPosition());
//            logData("frontLeft", frontLeftMotor.getCurrentPosition());
//            logData("frontRight", frontRightMotor.getCurrentPosition());
//            goRightDistance(DEFAULT_SIDEWAYS_SPEED, 1);
//            stopRobot();
//            sleep(500000);

            if (gamepad1.y) {
                if (power + 0.05 <= 1)
                    power += 0.05;
            } else if (gamepad1.a) {
                if (power - 0.05 >= 0)
                    power -= 0.05;
            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y)
            // ;
            encoderStartPos.put(backLeftMotor, Math.abs(backLeftMotor.getCurrentPosition()));
            backLeftMotor.setPower(power);
            sleep(waitTime);
            double currentRpm = ((double) (Math.abs(backLeftMotor.getCurrentPosition()) - encoderStartPos.get(backLeftMotor)) / wheelEncoderPpr) / (waitTime / 60000.0);
            logData("power", power);
            logData("rpm", currentRpm);
            updateTelemetry();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
