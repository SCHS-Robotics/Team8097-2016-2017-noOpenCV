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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

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

@Autonomous(name = "Range Test", group = "Test")
public class RangeTest extends BaseOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

//        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
//        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
//        rightRangeSensor.setI2cAddress(rightRangeI2c);
//        rangeSensor.setI2cAddress(rangeI2c);

        I2cDevice rangeDevice = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch rangeReader = new I2cDeviceSynchImpl(rangeDevice, rangeI2c, false);
        rangeSensor = new RangeSensor(rangeReader);
        rangeServo = hardwareMap.servo.get("rangeServo");

//        rightRangeReader.engage();
//        leftRangeReader.engage();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runtime.reset();
//            logData("Left Distance", rangeSensor.getDistance(DistanceUnit.INCH) + " inches");
//            logData("Right Distance", rightRangeSensor.getDistance(DistanceUnit.INCH) + " inches");

//            int rightUltrasonic = rightRangeReader.read(0x04, 1)[0];
//            int leftUltrasonic = leftRangeReader.read(0x04, 1)[0];

            logData("degrees", determineAngleOffset());

            //display values
            updateTelemetry();
            sleep(10000000);

//            sleep((int) (50 - runtime.time()));

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public double determineAngleOffset() throws InterruptedException {
        double angleWindow = 60;
        double startPos = rangeServoInitPos - ((angleWindow / 2) * SERVO_POS_PER_DEGREE);
        double angleIncrement = 2;
        int numReads = (int) Math.round(angleWindow / angleIncrement);
        int[] distances = new int[numReads];
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
            distances[i] = ultrasonic;
            if (ultrasonic < minDistance) {
                minDistance = ultrasonic;
                firstMinPos = pos;
            } else if (ultrasonic == minDistance) {
                lastMinPos = pos;
            }
        }
        rangeServo.setPosition(((lastMinPos + firstMinPos) / 2));
        return (((lastMinPos + firstMinPos) / 2) - 0.5) / SERVO_POS_PER_DEGREE;
    }
}
