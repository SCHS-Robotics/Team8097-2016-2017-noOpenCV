package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "CDI Sensor Test", group = "Test")
public class CDISensorTest extends OpMode {//CDI stands for Core Device Interface
    //    private OpticalDistanceSensor ods;
//    private LightSensor lightSensor;
    private ColorSensor leftColorSensor;
    private ColorSensor rightColorSensor;


    @Override
    public void init() {
        leftColorSensor = hardwareMap.colorSensor.get("leftColor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColor");
    }

    @Override
    public void loop() {
        telemetry.addData("Left Red", leftColorSensor.red());
        telemetry.addData("Left Green", leftColorSensor.green());
        telemetry.addData("Left Blue", leftColorSensor.blue());

        telemetry.addData("Right Red", rightColorSensor.red());
        telemetry.addData("Right Green", rightColorSensor.green());
        telemetry.addData("Right Blue", rightColorSensor.blue());
    }
}
