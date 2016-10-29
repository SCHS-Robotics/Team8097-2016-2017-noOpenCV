package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Autonomous(name = "CDI Sensor Test", group = "Testing")
public class CDISensorTest extends OpMode {//CDI stands for Core Device Interface
    //    private OpticalDistanceSensor ods;
//    private LightSensor lightSensor;
    private ColorSensor colorSensor;


    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(false);
    }

    @Override
    public void loop() {
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
    }
}
