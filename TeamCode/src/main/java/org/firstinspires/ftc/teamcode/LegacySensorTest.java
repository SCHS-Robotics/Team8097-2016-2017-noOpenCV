package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Legacy Sensor Test", group = "OpMode")
public class LegacySensorTest extends OpMode {//CDI stands for Core Device Interface
    //    private OpticalDistanceSensor ods;
//    private LightSensor lightSensor;
    private ColorSensor colorSensor;


    @Override
    public void init() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
    }
}
