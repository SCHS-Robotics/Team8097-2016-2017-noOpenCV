package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumTest", group = "Test")
public class TestMecanum extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotorback;
    private DcMotor rightMotorback;
    private DcMotor leftMotorfront;
    private DcMotor rightMotorfront;
//     private DcMotor leftMotor = null;
//     private DcMotor rightMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftMotorback = hardwareMap.dcMotor.get("LeftMotorBack");
        rightMotorback = hardwareMap.dcMotor.get("RightMotorBack");
        leftMotorfront = hardwareMap.dcMotor.get("LeftMotorFront");
        rightMotorfront = hardwareMap.dcMotor.get("RightMotorFront");
        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        leftMotorback.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
        rightMotorback.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2);
        leftMotorfront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);
        rightMotorfront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2);

        if (gamepad1.dpad_right) {
            leftMotorback.setPower(-1);
            rightMotorback.setPower(-1);
            leftMotorfront.setPower(-1);
            rightMotorfront.setPower(-1);
        } else if (gamepad1.dpad_left) {
            leftMotorback.setPower(1);
            rightMotorback.setPower(1);
            leftMotorfront.setPower(1);
            rightMotorfront.setPower(1);
        }
    }

    // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
    // leftMotor.setPower(-gamepad1.left_stick_y);
    // rightMotor.setPower(-gamepad1.right_stick_y);


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
