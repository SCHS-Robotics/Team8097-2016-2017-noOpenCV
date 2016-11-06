package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.HashMap;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_POWER = 0.5;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    Servo rightFlapServo;
    Servo leftFlapServo;
    ColorSensor rightColorSensor;
    ColorSensor leftColorSensor;
    ColorSensor frontTapeSensor;
    ColorSensor backTapeSensor;
    I2cAddr leftColorI2c = I2cAddr.create8bit(0x3c);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr frontTapeI2c = I2cAddr.create8bit(0x5c);
//    I2cAddr backTapeI2c = I2cAddr.create8bit(0x6c);
    ModernRoboticsI2cRangeSensor leftRangeSensor;
    ModernRoboticsI2cRangeSensor rightRangeSensor;
    I2cAddr rightRangeI2c = I2cAddr.create8bit(0x28);
    I2cAddr leftRangeI2c = I2cAddr.create8bit(0x38);

    double leftFlapInitPos = 0.704;
    double rightFlapInitPos = 0.114;
    double leftFlapEndPos = 0.466;
    double rightFlapEndPos = 0.352;

    private HashMap<String, String> telemetryData = new HashMap<String, String>();

    protected void logData(String label, String value) {
        telemetry.addData(label, value);
        telemetryData.put(label, value);
        String data = "";
        for (String key : telemetryData.keySet()) {
            data += key + ": " + telemetryData.get(key) + "" + "\n";
        }
        FtcRobotControllerActivity.logData.obtainMessage(0, data).sendToTarget();
    }

    public void resetEncoders() {
        while (motorFrontRight.getCurrentPosition() != 0)
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorBackRight.getCurrentPosition() != 0)
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorFrontLeft.getCurrentPosition() != 0)
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorBackLeft.getCurrentPosition() != 0)
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void allInit() {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
        rightRangeSensor.setI2cAddress(rightRangeI2c);
        leftRangeSensor.setI2cAddress(leftRangeI2c);

        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        frontTapeSensor.enableLed(true);
        backTapeSensor.enableLed(true);

        rightColorSensor = hardwareMap.colorSensor.get("rightColor");
        leftColorSensor = hardwareMap.colorSensor.get("leftColor");
        rightColorSensor.setI2cAddress(rightColorI2c);
        leftColorSensor.setI2cAddress(leftColorI2c);
        rightColorSensor.enableLed(false);
        leftColorSensor.enableLed(false);

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);
    }
}
