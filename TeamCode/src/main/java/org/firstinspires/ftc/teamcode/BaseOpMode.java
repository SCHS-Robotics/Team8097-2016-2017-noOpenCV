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

    public final static double DEFAULT_FORWARD_SPEED = 0.9;
    public final static double DEFAULT_DIAGONAL_SPEED = 0.9;
    public final static double DEFAULT_SIDEWAYS_SPEED = 1;
    public final static double DEFAULT_SPIN_SPEED = 1;

    public final static double TICKS_PER_CM_FORWARD = 53.565;
    public final static double TICKS_PER_CM_SIDEWAYS = 70.304;
    public final static double TICKS_PER_CM_DIAGONAL = 88.348;
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
    int wheelMaxRpm = 103;
    double theConstant = 9.5;
    int launcherMaxRpm = 1500;//theoretical 1650
    Servo rightFlapServo;
    Servo leftFlapServo;
    Servo rangeServo;
    Servo launcherServo;
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
    double rangeServoInitPos = 0.460;
    double launcherServoInitPos = 0;//TODO

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

    public void stopRobot() {
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    public void spinRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void spinLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goForward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
    }

    public void goBackward(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
    }

    public void goLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(-power);
    }

    public void goRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalForwardRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(-power);
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(0);
    }

    public void goDiagonalForwardLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(-power);
    }

    public void goDiagonalBackwardRight(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(power);
    }

    public void goDiagonalBackwardLeft(double percentSpeed) {
        double power = getPowerEstimateFromPercentSpeed(percentSpeed);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(power);
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(0);
    }

    public void resetWheelEncoders() {
        resetEncoders(backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor);
    }

    public void resetWheelEncoderStartPos() {
        encoderStartPos.put(backLeftMotor, backLeftMotor.getCurrentPosition());
        encoderStartPos.put(backRightMotor, backRightMotor.getCurrentPosition());
        encoderStartPos.put(frontLeftMotor, frontLeftMotor.getCurrentPosition());
        encoderStartPos.put(frontRightMotor, frontRightMotor.getCurrentPosition());
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

    public double getRpmEstimate(double power) {
        return Math.max((wheelMaxRpm + theConstant) - (theConstant / Math.abs(power)), 0);
    }

    public double getPowerEstimateFromPercentSpeed(double percentSpeed) {
        double rpm = Math.abs(percentSpeed) * wheelMaxRpm;
        return getPowerEstimate(rpm) * Math.signum(percentSpeed);
    }

    public double getPowerEstimate(double rpm) {
        if (rpm < 10) {
            return 0;
        }
        rpm = Math.min(wheelMaxRpm, rpm);
        return -theConstant / (rpm - (wheelMaxRpm + theConstant));
    }

    public double applyPercentErrorToWheelPower(double power, double percentError) {
        double estimatedRpm = getRpmEstimate(power);
        double modifiedRpm = estimatedRpm / (percentError + 1);
        return Math.signum(power) * getPowerEstimate(modifiedRpm);
    }

    public double reversePercentErrorOnWheelPower(double resultPower, double percentError) {
        double modifiedRpm = getRpmEstimate(resultPower);
        double originalEstimatedRpm = modifiedRpm * (percentError + 1);
        return Math.signum(resultPower) * getPowerEstimate(originalEstimatedRpm);
    }

    public double[] fixRpm(double rpm, int encoderPpr, DcMotor... motors) throws InterruptedException {
        final int waitTime = 250;
        double[] percentErrors = new double[motors.length];
        Arrays.fill(percentErrors, -2);
        if (fixRpmTimer.time() >= waitTime) {
            fixRpmTimer.reset();
            for (int i = 0; i < motors.length; i++) {
                DcMotor motor = motors[i];
                double currentRpm = getCurrentRpm(encoderPpr, motor, waitTime);
                percentErrors[i] = getPercentError(rpm, currentRpm);
                double newPower;
                if (encoderPpr == wheelEncoderPpr) {
                    logData("percent error " + i, percentErrors[i]);
                    double expectedRpm = getRpmEstimate(motor.getPower());
                    double modifiedRpm = expectedRpm / (percentErrors[i] + 1);
                    logData("modified rpm " + i, modifiedRpm);
                    newPower = Math.signum(motor.getPower()) * getPowerEstimate(modifiedRpm);
                    logData("new power " + i, newPower);
                } else {
                    newPower = motor.getPower() / (percentErrors[i] + 1);
                }
                encoderStartPos.put(motor, Math.abs(motor.getCurrentPosition()));
                motor.setPower(newPower);
            }
            updateTelemetry();
        }
        return percentErrors;
    }

    public double getCurrentRpm(int encoderPpr, DcMotor motor, int waitTime) {
        return ((double) (Math.abs(motor.getCurrentPosition()) - encoderStartPos.get(motor)) / encoderPpr) / (waitTime / 60000.0);
    }

    public double getPercentError(double goalRpm, double currentRpm) {
        return (currentRpm - goalRpm) / goalRpm;
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
        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectionMotor = hardwareMap.dcMotor.get("collection");

//        I2cDevice rightRangeDevice = hardwareMap.i2cDevice.get("rightRange");
        I2cDevice rangeDevice = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch rightRangeReader = new I2cDeviceSynchImpl(rightRangeDevice, rightRangeI2c, false);
        I2cDeviceSynch rangeReader = new I2cDeviceSynchImpl(rangeDevice, rangeI2c, false);
//        rightRangeSensor = new RangeSensor(rightRangeReader);
        rangeSensor = new RangeSensor(rangeReader);

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
        rangeServo = hardwareMap.servo.get("rangeServo");
        rangeServo.setPosition(rangeServoInitPos);
        launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(launcherServoInitPos);
    }
}
