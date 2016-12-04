package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test Encoders", group = "Test")
public class TestEncoders extends AutonomousOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    double power = 0;
    int waitTime = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

//        allInit();
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");

        waitForStart();
        runtime.reset();
        resetWheelEncoders();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            goRightDistance(DEFAULT_SIDEWAYS_SPEED, 1);
            stopRobot();
            sleep(500000);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
