package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Servos", group = "Util")
public class TestServos extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double pos1 = 0.5;
    double pos2 = 0.5;
    double pos3 = 0.5;
    double pos4 = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFlapServo = hardwareMap.servo.get("leftFlap");
        rightFlapServo = hardwareMap.servo.get("rightFlap");
        leftFlapServo.setPosition(0.5);
        rightFlapServo.setPosition(0.5);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if (gamepad1.y) {
                if (pos1 + 0.002 <= 1)
                    pos1 += 0.002;
            } else if (gamepad1.a) {
                if (pos1 - 0.002 >= 0)
                    pos1 -= 0.002;
            }
            if (gamepad1.b) {
                if (pos2 + 0.002 <= 1)
                    pos2 += 0.002;
            } else if (gamepad1.x) {
                if (pos2 - 0.002 >= 0)
                    pos2 -= 0.002;
            }
            if (gamepad1.dpad_up) {
                if (pos3 + 0.002 <= 1)
                    pos3 += 0.002;
            } else if (gamepad1.dpad_down) {
                if (pos3 - 0.002 >= 0)
                    pos3 -= 0.002;
            }
            if (gamepad1.dpad_right) {
                if (pos4 + 0.002 <= 1)
                    pos4 += 0.002;
            } else if (gamepad1.dpad_left) {
                if (pos4 - 0.002 >= 0)
                    pos4 -= 0.002;
            }
            leftFlapServo.setPosition(pos1);
            telemetry.addData("leftFlap", String.valueOf(pos1));
            rightFlapServo.setPosition(pos2);
            telemetry.addData("rightFlap", String.valueOf(pos2));
            sleep(10);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
