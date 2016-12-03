package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Calibrate Launcher", group = "Util")
public class TestLauncher extends BaseOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launcherTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double pos = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        leftLaunchMotor = hardwareMap.dcMotor.get("leftLaunch");
        rightLaunchMotor = hardwareMap.dcMotor.get("rightLaunch");
        launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(0);

        waitForStart();
        runtime.reset();
        launcherTime.reset();
        resetEncoders(rightLaunchMotor, leftLaunchMotor);
        launchMotorsRpm(1400);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            logData("Status", "Run Time: " + runtime.toString());

            if (gamepad1.y) {
                if (pos + 0.002 <= 1)
                    pos += 0.002;
            } else if (gamepad1.a) {
                if (pos - 0.002 >= 0)
                    pos -= 0.002;
            }
            logData("position", pos);
            updateTelemetry();
            launcherServo.setPosition(pos);
            sleep(10);


            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

    }

    public void launchMotorsRpm(double rpm) throws InterruptedException {
        final int waitTime = 250;
        double leftPercentError = 0;
        double rightPercentError = 0;
//        resetEncoders(leftLaunchMotor, rightLaunchMotor);
        int leftStartPos = 0;
        int rightStartPos = 0;
        leftLaunchMotor.setPower(-0.5);
        rightLaunchMotor.setPower(0.5);
        sleep(waitTime);
        while ((Math.abs(1 - leftPercentError) > 0.015 || (Math.abs(1 - rightPercentError) > 0.015)) && opModeIsActive()) {
            launcherTime.reset();

            double currentLeftRpm = ((Math.abs(leftLaunchMotor.getCurrentPosition()) - leftStartPos) / 112.0) / (waitTime / 60000.0);
            leftPercentError = currentLeftRpm / rpm;
            logData("left encoder", Math.abs(leftLaunchMotor.getCurrentPosition()) - leftStartPos);
            logData("leftPercentError", leftPercentError);
            leftStartPos = Math.abs(leftLaunchMotor.getCurrentPosition());
            leftLaunchMotor.setPower(-(Math.abs(leftLaunchMotor.getPower()) / leftPercentError));
            logData("left power", leftLaunchMotor.getPower());

            double currentRightRpm = ((Math.abs(rightLaunchMotor.getCurrentPosition()) - rightStartPos) / 112.0) / (waitTime / 60000.0);
            rightPercentError = currentRightRpm / rpm;
            logData("right encoder", Math.abs(rightLaunchMotor.getCurrentPosition()) - rightStartPos);
            logData("rightPercentError", rightPercentError);
            rightStartPos = Math.abs(rightLaunchMotor.getCurrentPosition());
            rightLaunchMotor.setPower((Math.abs(rightLaunchMotor.getPower()) / rightPercentError));
            logData("right power", rightLaunchMotor.getPower());

            updateTelemetry();

            if (waitTime - launcherTime.time() > 0) {
                sleep((int) (waitTime - launcherTime.time()));
            }
        }
    }
}
