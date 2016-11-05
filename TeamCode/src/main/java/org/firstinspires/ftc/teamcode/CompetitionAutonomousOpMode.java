package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;


public abstract class CompetitionAutonomousOpMode extends AutonomousOpMode {
    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    public void initialize() {
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");

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

        frontTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("frontTapeValue", -2) + FtcRobotControllerActivity.calibrationSP.getFloat("frontGroundValue", -2)) / 2.0;
        if (frontTapeLowThreshold < 0) {
            frontTapeLowThreshold = 0.43;//TODO Find reasonable default value
        }
        backTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("backTapeValue", -2) + FtcRobotControllerActivity.calibrationSP.getFloat("backGroundValue", -2)) / 2.0;
        if (backTapeLowThreshold < 0) {
            backTapeLowThreshold = 0.43;//TODO Find reasonable default value
        }
    }
}