package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Range Test", group = "Test")
public class RangeTest extends Autonomous {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

//        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightRange");
//        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftRange");
//        rightRangeSensor.setI2cAddress(rightRangeI2c);
//        rangeSensor.setI2cAddress(rangeI2c);

        I2cDevice rangeDevice = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch rangeReader = new I2cDeviceSynchImpl(rangeDevice, rangeI2c, false);
        rangeSensor = new RangeSensor(rangeReader);
        rangeServo = hardwareMap.servo.get("rangeServo");

//        rightRangeReader.engage();
//        leftRangeReader.engage();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runtime.reset();
//            logData("Left Distance", rangeSensor.getDistance(DistanceUnit.INCH) + " inches");
//            logData("Right Distance", rightRangeSensor.getDistance(DistanceUnit.INCH) + " inches");

//            int rightUltrasonic = rightRangeReader.read(0x04, 1)[0];
//            int leftUltrasonic = leftRangeReader.read(0x04, 1)[0];

            logData("degrees", determineAngleOffset());

            //display values
            updateTelemetry();
            sleep(10000000);

//            sleep((int) (50 - runtime.time()));

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
