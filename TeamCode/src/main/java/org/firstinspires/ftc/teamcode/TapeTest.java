package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Tape Test", group = "Test")
public class TapeTest extends BaseOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        frontTapeSensor = hardwareMap.colorSensor.get("frontTape");
        frontTapeSensor.setI2cAddress(frontTapeI2c);
        frontTapeSensor.enableLed(true);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());

            logData("Light", frontTapeSensor.alpha());

            updateTelemetry();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
