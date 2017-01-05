package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_FORWARD_SPEED = 1;
    public final static double DEFAULT_DIAGONAL_SPEED = 0.85;
    public final static double DEFAULT_SIDEWAYS_SPEED = 1;
    public final static double DEFAULT_SPIN_SPEED = 1;

    public final static double TICKS_PER_CM_FORWARD = 53.6;
    public final static double TICKS_PER_CM_SIDEWAYS = 72.0;
    public final static double TICKS_PER_CM_DIAGONAL = 88.3;
    public final static double TICKS_PER_DEGREE = 27.777;

    public final static double SERVO_POS_PER_DEGREE = 1.0 / 151.0;

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftLaunchMotor;
    DcMotor rightLaunchMotor;
    DcMotor collectionMotor;
    HashMap<DcMotor, Integer> encoderStartPos = new HashMap<>();
    int wheelEncoderPpr = 1680;
    int launcherEncoderPpr = 112;
    int wheelMaxRpm = 100;//theoretical 110
    int launcherMaxRpm = 1400;//theoretical 1650
    Servo rightFlapServo;
    Servo leftFlapServo;
    Servo rangeServo;
    Servo launcherServo;
    Servo leftLiftServo;
    Servo rightLiftServo;
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

    double leftFlapInitPos = 0.710;
    double rightFlapInitPos = 0.324;
    double leftFlapEndPos = 0.512;
    double rightFlapEndPos = 0.502;
    double rangeServoInitPos = 0.518;
    double launcherServoAutoPos = 0.816;
    double launcherServoShortPos;//TODO
    double launcherServoFarPos = launcherServoAutoPos;//TODO
    double launcherServoInitPos = launcherServoAutoPos;
    double leftLiftInitPos = 0.636;
    double rightLiftInitPos = 0.210;
    double leftLiftEndPos = 0.354;
    double rightLiftEndPos = 0.664;

    int launchFarServoWaitTime = 38;//milliseconds
    int launchShortServoWaitTime = 38;//TODO milliseconds

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

    public void startLauncher() {
        leftLaunchMotor.setPower(-1);
        rightLaunchMotor.setPower(1);
    }

    public void stopLauncher() {
        leftLaunchMotor.setPower(0);
        rightLaunchMotor.setPower(0);
    }

    public void liftToLaunch() throws InterruptedException {
        leftLiftServo.setPosition(leftLiftEndPos);
        if (launcherServo.getPosition() == launcherServoShortPos) {
            sleep(launchShortServoWaitTime);
        } else {
            sleep(launchFarServoWaitTime);
        }
        rightLiftServo.setPosition(rightLiftEndPos);
    }

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void spinRight(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    public void spinLeft(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
    }

    public void goForward(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
    }

    public void goBackward(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
    }

    public void goLeft(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
    }

    public void goRight(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
    }

    public void goDiagonalForwardRight(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-speed);
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(0);
    }

    public void goDiagonalForwardLeft(double speed) {
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-speed);
    }

    public void goDiagonalBackwardRight(double speed) {
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(speed);
    }

    public void goDiagonalBackwardLeft(double speed) {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(speed);
        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(0);
    }

    public void resetWheelEncoders() {
        resetEncoders(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void resetEncoders(DcMotor... motors) {
        for (DcMotor motor : motors) {
            while (motor.getCurrentPosition() != 0)
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderStartPos.put(motor, 0);
        }
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getCurrentRpm(int encoderPpr, DcMotor motor, int waitTime) {
        return ((double) (Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
    }

    public void initWheels() {
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backLeftMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        backRightMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        frontLeftMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        frontRightMotor.setMaxSpeed((int) ((wheelMaxRpm * wheelEncoderPpr) / 60.0));
        resetWheelEncoders();
    }

    public void initLauncher() {
        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        leftLaunchMotor.setMaxSpeed((int) ((launcherMaxRpm * launcherEncoderPpr) / 60.0));
        rightLaunchMotor.setMaxSpeed((int) ((launcherMaxRpm * launcherEncoderPpr) / 60.0));
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetEncoders(leftLaunchMotor, rightLaunchMotor);

        launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(launcherServoInitPos);
        leftLiftServo = hardwareMap.servo.get("leftLift");
        rightLiftServo = hardwareMap.servo.get("rightLift");
        leftLiftServo.setPosition(leftLiftInitPos);
        rightLiftServo.setPosition(rightLiftInitPos);
    }

    public void initCollection() {
        collectionMotor = hardwareMap.dcMotor.get("collect");
    }

    public void initRange() {
        //        I2cDevice rightRangeDevice = hardwareMap.i2cDevice.get("rightRange");
        I2cDevice rangeDevice = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch rightRangeReader = new I2cDeviceSynchImpl(rightRangeDevice, rightRangeI2c, false);
        I2cDeviceSynch rangeReader = new I2cDeviceSynchImpl(rangeDevice, rangeI2c, false);
//        rightRangeSensor = new RangeSensor(rightRangeReader);
        rangeSensor = new RangeSensor(rangeReader);

        rangeServo = hardwareMap.servo.get("rangeServo");
        rangeServo.setPosition(rangeServoInitPos);
    }

    public void initTape() {
        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
//        backTapeSensor = hardwareMap.colorSensor.get("backTape");
        frontTapeSensor.setI2cAddress(frontTapeI2c);
        frontTapeSensor.enableLed(true);
//        backTapeSensor.enableLed(true);
    }

    public void initButtons() {
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


    public void allInit() {
        initWheels();
        initLauncher();
        initCollection();
        initRange();
        initTape();
        initButtons();
    }
}
