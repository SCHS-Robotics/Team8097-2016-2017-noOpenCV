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

import java.util.Arrays;
import java.util.HashMap;

public abstract class BaseOpMode extends LinearOpMode {

    public final static double DEFAULT_FORWARD_POWER = 0.5;
    public final static double DEFAULT_DIAGONAL_POWER = 0.5;
    public final static double DEFAULT_SIDEWAYS_POWER = 1;

    public final static double TICKS_PER_CM_FORWARD = 53.565;
    public final static double TICKS_PER_CM_SIDEWAYS = 70.304;
    public final static double TICKS_PER_CM_DIAGONAL = 88.348;

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftLaunchMotor;
    DcMotor rightLaunchMotor;
    HashMap<DcMotor, Integer> encoderStartPos = new HashMap<DcMotor, Integer>() {{
        put(backLeftMotor, 0);
        put(backRightMotor, 0);
        put(frontLeftMotor, 0);
        put(frontRightMotor, 0);
        put(leftLaunchMotor, 0);
        put(rightLaunchMotor, 0);
    }};
    HashMap<DcMotor, Integer> pulsesPerRevolution = new HashMap<DcMotor, Integer>() {{
        put(backLeftMotor, 420);
        put(backRightMotor, 420);
        put(frontLeftMotor, 420);
        put(frontRightMotor, 420);
        put(leftLaunchMotor, 112);
        put(rightLaunchMotor, 112);
    }};
    int wheelMaxRpm = 105;//TODO test for experimental max
    int launcherMaxRpm = 1500;//theoretical 1650
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

    ElapsedTime fixRpmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private HashMap<String, Object> telemetryData = new HashMap<String, Object>();

    protected void logData(String label, Object value) {
        telemetry.addData(label, value);
        telemetryData.put(label, value);
        String data = "";
        for (String key : telemetryData.keySet()) {
            data += key + ": " + telemetryData.get(key) + "" + "\n";
        }
        FtcRobotControllerActivity.logData.obtainMessage(0, data).sendToTarget();
    }

    protected void spinRight(double power) {
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    protected void spinLeft(double power) {
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
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

    public double[] fixRpm(double rpm, DcMotor... motors) throws InterruptedException {
        final int waitTime = 250;
        double[] percentErrors = new double[motors.length];
        Arrays.fill(percentErrors, -2);
        if (fixRpmTimer.time() >= waitTime) {
            fixRpmTimer.reset();
            for (int i = 0; i < motors.length; i++) {
                DcMotor motor = motors[i];
                int direction = (int) Math.signum(motor.getPower());
                double currentRpm = ((Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / pulsesPerRevolution.get(motor)) / (waitTime / 60000.0);
                percentErrors[i] = (currentRpm - rpm) / rpm;
                encoderStartPos.put(motor, Math.abs(motor.getCurrentPosition()));
                motor.setPower(direction * (Math.abs(motor.getPower()) / (percentErrors[i] + 1)));
            }
        }
        return percentErrors;
    }

    public void startLauncher(double rpm) throws InterruptedException {
        resetLauncherEncoders();
        fixRpmTimer.reset();
        leftLaunchMotor.setPower(rpm / launcherMaxRpm);
        rightLaunchMotor.setPower(rpm / launcherMaxRpm);
        boolean done = false;
        while (!done) {
            double[] percentErrors = fixRpm(rpm, leftLaunchMotor, rightLaunchMotor);
            done = Math.abs(percentErrors[0]) <= 0.15 && Math.abs(percentErrors[1]) <= 0.15;
        }
    }

    public void allInit() {
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

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
