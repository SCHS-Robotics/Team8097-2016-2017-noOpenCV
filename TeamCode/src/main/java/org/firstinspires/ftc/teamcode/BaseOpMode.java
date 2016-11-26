package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.HashMap;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_FORWARD_POWER = 0.5;
    public final static double DEFAULT_DIAGONAL_POWER = 0.5;
    public final static double DEFAULT_SIDEWAYS_POWER = 1;

    public final static double TICKS_PER_CM_FORWARD = 53.565;
    public final static double TICKS_PER_CM_SIDEWAYS = 70.304;
    public final static double TICKS_PER_CM_DIAGONAL = 88.348;
    public final static double TICKS_PER_DEGREE = 10000;//TODO

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftLaunchMotor;
    DcMotor rightLaunchMotor;
    HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();
    int wheelEncoderPpr = 420;//TODO test
    int launcherEncoderPpr = 112;
    int wheelMaxRpm = 105;//TODO test for experimental max
    int launcherMaxRpm = 1500;//theoretical 1650
    Servo rightFlapServo;
    Servo leftFlapServo;
    ColorSensor rightColorSensor;
    ColorSensor leftColorSensor;
    ColorSensor frontTapeSensor;
    //    ColorSensor backTapeSensor;
    I2cAddr leftColorI2c = I2cAddr.create8bit(0x3c);
    I2cAddr rightColorI2c = I2cAddr.create8bit(0x4c);
    I2cAddr frontTapeI2c = I2cAddr.create8bit(0x5c);
    //    I2cAddr backTapeI2c = I2cAddr.create8bit(0x6c);
    RangeSensor rangeSensor;
    //    RangeSensor rightRangeSensor;
    I2cAddr rangeI2c = I2cAddr.create8bit(0x28);
//    I2cAddr rightRangeI2c = I2cAddr.create8bit(0x38);

    double leftFlapInitPos = 0.324;
    double rightFlapInitPos = 0.780;
    double leftFlapEndPos = 0.162;
    double rightFlapEndPos = 0.936;

    ElapsedTime fixRpmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private HashMap<String, Object> telemetryData = new HashMap<String, Object>();

    public void logData(String label, Object value) {
        telemetry.addData(label, value);
        telemetryData.put(label, value);
    }

    public void updateTelemetry() {
        updateTelemetry(telemetry);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
        if (!FtcRobotControllerActivity.logData.hasMessages(0)) {
            StringBuilder data = new StringBuilder();
            for (String key : telemetryData.keySet()) {
                data.append(key).append(": ").append(telemetryData.get(key)).append("\n");
            }
            FtcRobotControllerActivity.logData.obtainMessage(0, data.toString()).sendToTarget();
            telemetryData.clear();
        }
    }

    public void resetWheelEncoders() {
        resetEncoders(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void resetLauncherEncoders() {
        resetEncoders(leftLaunchMotor, rightLaunchMotor);
    }

    public void resetEncoders(DcMotor... motors) {
        for (DcMotor motor : motors) {
            while (motor.getCurrentPosition() != 0)
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderStartPos.put(motor, 0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double[] fixRpm(double rpm, int encoderPpr, DcMotor... motors) throws InterruptedException {
        final int waitTime = 250;
        double[] percentErrors = new double[motors.length];
        Arrays.fill(percentErrors, -2);
        if (fixRpmTimer.time() >= waitTime) {
            fixRpmTimer.reset();
            for (int i = 0; i < motors.length; i++) {
                DcMotor motor = motors[i];
                percentErrors[i] = getPercentError(rpm, encoderPpr, motor, waitTime);
                encoderStartPos.put(motor, Math.abs(motor.getCurrentPosition()));
                motor.setPower(motor.getPower() / (percentErrors[i] + 1));
            }
        }
        return percentErrors;
    }

    public double getPercentError(double rpm, int encoderPpr, DcMotor motor, int waitTime) {
        double currentRpm = ((Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
        return (currentRpm - rpm) / rpm;
    }

    public void startLauncher(double rpm) throws InterruptedException {
        resetLauncherEncoders();
        fixRpmTimer.reset();
        leftLaunchMotor.setPower(rpm / launcherMaxRpm);
        rightLaunchMotor.setPower(rpm / launcherMaxRpm);
        boolean done = false;
        while (!done) {
            double[] percentErrors = fixRpm(rpm, launcherEncoderPpr, leftLaunchMotor, rightLaunchMotor);
            done = Math.abs(percentErrors[0]) <= 0.15 && Math.abs(percentErrors[1]) <= 0.15;
        }
    }

    public void allInit() {
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

//        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
//        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
//        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        I2cDevice rightRangeDevice = hardwareMap.i2cDevice.get("rightRange");
        I2cDevice leftRangeDevice = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch rightRangeReader = new I2cDeviceSynchImpl(rightRangeDevice, rightRangeI2c, false);
        I2cDeviceSynch leftRangeReader = new I2cDeviceSynchImpl(leftRangeDevice, rangeI2c, false);
//        rightRangeSensor = new RangeSensor(rightRangeReader);
        rangeSensor = new RangeSensor(leftRangeReader);

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
