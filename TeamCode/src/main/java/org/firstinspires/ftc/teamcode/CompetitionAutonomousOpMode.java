package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public abstract class CompetitionAutonomousOpMode extends AutonomousOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    double frontTapeLowThreshold;
    double backTapeLowThreshold;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //do stuff

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void initialize() {
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

        frontTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("frontTapeValue", -2) + FtcRobotControllerActivity.calibrationSP.getFloat("frontGroundValue", -2)) / 2.0;
        if (frontTapeLowThreshold < 0) {
            frontTapeLowThreshold = 10;
        }
        backTapeLowThreshold = (FtcRobotControllerActivity.calibrationSP.getFloat("backTapeValue", -2) + FtcRobotControllerActivity.calibrationSP.getFloat("backGroundValue", -2)) / 2.0;
        if (backTapeLowThreshold < 0) {
            backTapeLowThreshold = 10;
        }
    }

    //These movements are with respect to the field. Different for red and blue because they mirror each other.
    public abstract void moveAcrossField(double power);

    public abstract void moveAlongStartWall(double power);

    public abstract void moveAlongBeaconWall(double power);


    //These movements are with respect to the autonomous side of the robot.
    public void moveLeftSideForward(double power) {
        moveBackWheelsLeft(power);
    }

    public void moveLeftSideBackward(double power) {
        moveBackWheelsRight(power);
    }

    public void moveRightSideForward(double power) {
        moveFrontWheelsLeft(power);
    }

    public void moveRightSideBackward(double power) {
        moveFrontWheelsRight(power);
    }
}