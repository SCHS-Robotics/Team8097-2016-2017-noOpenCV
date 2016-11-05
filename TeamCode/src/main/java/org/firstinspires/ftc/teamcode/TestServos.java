package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servos", group = "OpMode")
public class TestServos extends OpMode {
    double pos1 = 0.5;
    double pos2 = 0.5;
    double pos3 = 0.5;
    double pos4 = 0.5;

    Servo leftFlap;
    Servo rightFlap;

    @Override
    public void init() {
        leftFlap = hardwareMap.servo.get("leftFlap");
        rightFlap = hardwareMap.servo.get("rightFlap");
        leftFlap.setPosition(0.5);
        rightFlap.setPosition(0.5);
    }

    @Override
    public void loop() {
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
        leftFlap.setPosition(pos1);
        telemetry.addData("leftFlap", String.valueOf(pos1));
        rightFlap.setPosition(pos2);
        telemetry.addData("rightFlap", String.valueOf(pos2));
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
