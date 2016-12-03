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
//        resetWheelEncoders();
        resetWheelEncoders();
        fixRpmTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            logData("Status", "Run Time: " + runtime.toString());
//            fixRpm(wheelMaxRpm, wheelEncoderPpr, backLeftMotor);

//            goForward(DEFAULT_FORWARD_SPEED);
//            logData("backLeft", backLeftMotor.getCurrentPosition());
//            logData("backRight", backRightMotor.getCurrentPosition());
//            logData("frontLeft", frontLeftMotor.getCurrentPosition());
//            logData("frontRight", frontRightMotor.getCurrentPosition());
            spinRightDegrees(1, 1);
            stopRobot();
            sleep(500000);

//            if (gamepad1.y) {
//                if (power + 0.1 <= 1)
//                    power += 0.1;
//            } else if (gamepad1.a) {
//                if (power - 0.1 >= 0)
//                    power -= 0.1;
//            }

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y)
            // ;
//            encoderStartPos.put(backLeftMotor, Math.abs(backLeftMotor.getCurrentPosition()));
//            encoderStartPos.put(backRightMotor, Math.abs(backRightMotor.getCurrentPosition()));
//            encoderStartPos.put(frontLeftMotor, Math.abs(frontLeftMotor.getCurrentPosition()));
//            encoderStartPos.put(frontRightMotor, Math.abs(frontRightMotor.getCurrentPosition()));
//            backLeftMotor.setPower(power);
//            backRightMotor.setPower(-power);
//            frontLeftMotor.setPower(power);
//            frontRightMotor.setPower(-power);
//            sleep(waitTime);
//            double backLeftCurrentRpm = ((double) (Math.abs(backLeftMotor.getCurrentPosition()) - encoderStartPos.get(backLeftMotor)) / wheelEncoderPpr) / (waitTime / 60000.0);
//            double backRightCurrentRpm = ((double) (Math.abs(backRightMotor.getCurrentPosition()) - encoderStartPos.get(backRightMotor)) / wheelEncoderPpr) / (waitTime / 60000.0);
//            double frontLeftCurrentRpm = ((double) (Math.abs(frontLeftMotor.getCurrentPosition()) - encoderStartPos.get(frontLeftMotor)) / wheelEncoderPpr) / (waitTime / 60000.0);
//            double frontRightCurrentRpm = ((double) (Math.abs(frontRightMotor.getCurrentPosition()) - encoderStartPos.get(frontRightMotor)) / wheelEncoderPpr) / (waitTime / 60000.0);
//            double averageRpm = (backLeftCurrentRpm + backRightCurrentRpm + frontLeftCurrentRpm + frontRightCurrentRpm) / 4.0;
//            logData("power", power);
//            logData("average rpm", averageRpm);
            updateTelemetry();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
