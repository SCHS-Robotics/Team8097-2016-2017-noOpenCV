package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name="ODS Test", group="Testing")
public class CDISensorTest extends OpMode {
    private OpticalDistanceSensor ods;


    @Override
    public void init() {
        ods = hardwareMap.opticalDistanceSensor.get("ods");
    }

    @Override
    public void loop() {
        telemetry.addData("ODS", ods.getLightDetected());
    }
}
