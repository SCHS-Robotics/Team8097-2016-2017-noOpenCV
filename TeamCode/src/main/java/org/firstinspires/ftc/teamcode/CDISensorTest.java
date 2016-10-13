package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "Light Sensor Test", group = "Testing")
public class CDISensorTest extends OpMode {//CDI stands for Core Device Interface
    //    private OpticalDistanceSensor ods;
    private LightSensor lightSensor;


    @Override
    public void init() {
        lightSensor = hardwareMap.lightSensor.get("lightSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Light", lightSensor.getLightDetected());
    }
}
