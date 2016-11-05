package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.ArrayList;
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
    ModernRoboticsI2cRangeSensor leftRangeSensor;
    ModernRoboticsI2cRangeSensor rightRangeSensor;

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

    private ArrayList<Byte> I2cAddresses = new ArrayList<>();
    public DeviceInterfaceModule DIM;

    private boolean isInI2cAddressList(I2cAddr i2cAddress) {
        return isInI2cAddressList((byte) i2cAddress.get8Bit());
    }

    private boolean isInI2cAddressList(byte i2cAddressByte) {
        for (byte b : I2cAddresses) {
            if (i2cAddressByte == b) {
                return true;
            }
        }
        return false;
    }

    public ColorSensor getColorSensor(String name) throws InterruptedException {
        ColorSensor colorSensor = hardwareMap.colorSensor.get(name);
        if (isInI2cAddressList(colorSensor.getI2cAddress())) {//TODO getI2cAddress() is not correct. It only returns what has been set using setI2cAddress() in the code, not the actual address of the device.
            byte i2cAddressByte = I2cAddressChange.COLOR_SENSOR_ORIGINAL_ADDRESS;
            while (isInI2cAddressList(i2cAddressByte)) {
                i2cAddressByte += 0x10;
            }
            int port = 0;//TODO print getConnectionInfo() to see if it has port number
            I2cAddressChange.changeI2CAddress(this, DIM, port, colorSensor.getI2cAddress(), i2cAddressByte, I2cAddressChange.COLOR_SENSOR_FIRMWARE_REV, I2cAddressChange.COLOR_SENSOR_SENSOR_ID);
        }
        return colorSensor;
    }

    public ModernRoboticsI2cRangeSensor getRangeSensor(String name) throws InterruptedException {
        ModernRoboticsI2cRangeSensor rangeSensor = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get(name));
        if (isInI2cAddressList(rangeSensor.getI2cAddress())) {//TODO getI2cAddress() is not correct. It only returns what has been set using setI2cAddress() in the code, not the actual address of the device.
            byte i2cAddressByte = (byte) I2cAddressChange.RANGE_SENSOR_ORIGINAL_ADDRESS.get8Bit();
            while (isInI2cAddressList(i2cAddressByte)) {
                i2cAddressByte += 0x10;
            }
            int port = 0;//TODO print getConnectionInfo() to see if it has port number
            I2cAddressChange.changeI2CAddress(this, DIM, port, rangeSensor.getI2cAddress(), i2cAddressByte, I2cAddressChange.RANGE_SENSOR_FIRMWARE_REV, I2cAddressChange.RANGE_SENSOR_SENSOR_ID);
        }
        return rangeSensor;
    }

    private void allInit() {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        rightRangeSensor = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("rightRange"));
        leftRangeSensor = new ModernRoboticsI2cRangeSensor(hardwareMap.i2cDeviceSynch.get("leftRange"));

        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        frontTapeSensor.enableLed(true);
        backTapeSensor.enableLed(true);

        rightColorSensor = hardwareMap.colorSensor.get("rightColor");
        leftColorSensor = hardwareMap.colorSensor.get("leftColor");
        rightColorSensor.enableLed(false);
        leftColorSensor.enableLed(false);

        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo.setPosition(rightFlapInitPos);
        leftFlapServo.setPosition(leftFlapInitPos);
    }
}
