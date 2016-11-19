package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_FORWARD_POWER = 0.5;
    public final static double DEFAULT_DIAGONAL_POWER = 0.3;
    public final static double DEFAULT_SIDEWAYS_POWER = 1;

    public final static double TICKS_PER_CM_FORWARD = 56.243;
    public final static double TICKS_PER_CM_SIDEWAYS = 68.709;
    public final static double TICKS_PER_CM_DIAGONAL = 93.294;

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
    I2cDevice leftRangeSensor;
    I2cDevice rightRangeSensor;
    I2cDeviceSynch rightRangeReader;
    I2cDeviceSynch leftRangeReader;
    I2cAddr rightRangeI2c = I2cAddr.create8bit(0x28);
    I2cAddr leftRangeI2c = I2cAddr.create8bit(0x38);

    double leftFlapInitPos = 0.324;
    double rightFlapInitPos = 0.780;
    double leftFlapEndPos = 0.162;
    double rightFlapEndPos = 0.936;

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

    protected void spinRight(double power) {
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    protected void spinLeft(double power) {
        motorBackLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
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

    public void allInit() {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        rightRangeSensor = hardwareMap.i2cDevice.get("rightRange");
        leftRangeSensor = hardwareMap.i2cDevice.get("leftRange");
        rightRangeReader = new I2cDeviceSynchImpl(rightRangeSensor, rightRangeI2c, false);
        leftRangeReader = new I2cDeviceSynchImpl(leftRangeSensor, leftRangeI2c, false);
        rightRangeReader.engage();
        leftRangeReader.engage();

        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
//        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        frontTapeSensor.setI2cAddress(frontTapeI2c);
        frontTapeSensor.enableLed(true);
//        backTapeSensor.enableLed(true);

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
