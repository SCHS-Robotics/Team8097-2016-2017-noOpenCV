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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Blue Autonomous", group = "OpMode")
public class BlueAutonomousOpMode extends CompetitionAutonomousOpMode {

    @Override
    public void moveAcrossField(double power) {
        goDiagonalBackwardLeft(power);
    }

    @Override
    public void moveAcrossFieldDistance(double power, double centimeters) {
        goDiagonalBackwardLeftDistance(power, centimeters);
    }

    @Override
    public void moveAlongBeaconWall(double power) {
        goForward(power);
    }

    @Override
    public void moveAlongBeaconWallDistance(double power, double centimeters) {
        goForwardDistance(power, centimeters);
    }

    @Override
    public void findTapeInward() {
        findTapeLeft();
    }

    @Override
    public void findTapeOutward() {
        findTapeRight();
    }

    @Override
    public void pushCorrectButton() throws InterruptedException {
        int[] colors = getAverageColor(leftColorSensor, rightColorSensor);
        int leftColor = colors[0];
        int rightColor = colors[1];
        double leftBlue = Color.blue(leftColor);
        double rightBlue = Color.blue(rightColor);
        double leftRed = Color.red(leftColor);
        double rightRed = Color.red(rightColor);
        if (leftBlue > rightBlue && leftRed < rightRed) {
            leftFlapServo.setPosition(leftFlapEndPos);
            rightFlapServo.setPosition(rightFlapInitPos);
        } else if (rightBlue > leftBlue && rightRed < leftRed) {
            rightFlapServo.setPosition(rightFlapEndPos);
            leftFlapServo.setPosition(leftFlapInitPos);
        }
        sleep(500);
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);
    }
}
