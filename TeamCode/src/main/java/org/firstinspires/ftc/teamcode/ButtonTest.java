package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Button Test", group = "Test")
public class ButtonTest extends BaseOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        leftColorSensor = hardwareMap.colorSensor.get("leftColor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColor");
        rightColorSensor.setI2cAddress(rightColorI2c);
        leftColorSensor.setI2cAddress(leftColorI2c);
        leftColorSensor.enableLed(false);
        rightColorSensor.enableLed(false);

        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo.setPosition(leftFlapInitPos);
        rightFlapServo.setPosition(rightFlapInitPos);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());
            logData("Left Color", "red: " + leftColorSensor.red() + "blue: " + leftColorSensor.blue());
            logData("Right Color", "red: " + rightColorSensor.red() + "blue: " + rightColorSensor.blue());

            if (leftColorSensor.red() > rightColorSensor.red() && leftColorSensor.blue() < rightColorSensor.blue()) {
                leftFlapServo.setPosition(leftFlapEndPos);
                rightFlapServo.setPosition(rightFlapInitPos);
            } else if (rightColorSensor.red() > leftColorSensor.red() && rightColorSensor.blue() < leftColorSensor.blue()) {
                rightFlapServo.setPosition(rightFlapEndPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }

            updateTelemetry();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
