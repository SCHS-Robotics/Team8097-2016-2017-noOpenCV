package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LearningTest extends OpMode {
    public DcMotor leftMotor;


    @Override
    public void init() {

    }

    @Override
    public void loop() {
telemetry.addData("speed", leftMotor.getPower());
    }
}
