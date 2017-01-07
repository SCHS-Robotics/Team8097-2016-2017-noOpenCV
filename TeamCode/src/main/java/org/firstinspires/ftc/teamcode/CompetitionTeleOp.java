package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "OpMode")
public class CompetitionTeleOp extends BaseOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pushButtonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime liftTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    final int waitTime = 100;

    public static double currentAngle = 90;//initialized in autonomous based on which one is run.
    boolean spun = false;
    public final static int RIGHT = 0;
    public final static int FORWARD_RIGHT = 45;
    public final static int FORWARD = 90;
    public final static int FORWARD_LEFT = 135;
    public final static int LEFT = 180;
    public final static int BACKWARD_LEFT = 225;
    public final static int BACKWARD = 270;
    public final static int BACKWARD_RIGHT = 315;

    public final static double MIN_SPEED = 0;

    boolean prevA = false;
    boolean prevX = false;

    double pos = launcherServoInitPos;
    ElapsedTime launchTestingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int launchTestingWaitTime = 250;
    double launchTestingRpm;

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        pushButtonTime.reset();
        liftTime.reset();

        resetEncoders(leftLaunchMotor, rightLaunchMotor);
        launchTestingTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Movement
            if (gamepad1.right_trigger > MIN_SPEED || gamepad1.left_trigger > MIN_SPEED) {
                if (!spun) {
                    resetWheelEncoders();
                }
                if (gamepad1.right_trigger > MIN_SPEED) {
                    spinRight(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > MIN_SPEED) {
                    spinLeft(gamepad1.left_trigger);
                }
                spun = true;
            } else {
                if (spun) {
                    double averageTicks = (backLeftMotor.getCurrentPosition() + backRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition() + frontRightMotor.getCurrentPosition()) / 4.0;
                    currentAngle += -averageTicks / TICKS_PER_DEGREE;
                }
                double joystickInputX = gamepad1.left_stick_x;
                double joystickInputY = -gamepad1.left_stick_y;
                double magnitude = Math.sqrt(Math.pow(joystickInputX, 2) + Math.pow(joystickInputY, 2));
                double angle = Math.toDegrees(Math.atan2(joystickInputY, joystickInputX));
                angle -= (currentAngle - 90);
                if (magnitude > MIN_SPEED) {
                    goDirection(magnitude, angle);
                } else {
                    stopRobot();
                }
                spun = false;
            }

            if (gamepad1.right_stick_button) {
                currentAngle = 90;
                logData("Angle calibrated!", "Angle calibrated!");
            }

            //Button Pushers
            if (gamepad1.left_bumper) {
                pushButtonTime.reset();
                leftFlapServo.setPosition(leftFlapEndPos);
                rightFlapServo.setPosition(rightFlapEndPos);
            } else if (gamepad1.right_bumper) {
                pushButtonTime.reset();
                rightFlapServo.setPosition(rightFlapEndPos);
                leftFlapServo.setPosition(leftFlapEndPos);
            }
            if (pushButtonTime.time() >= 500) {
                rightFlapServo.setPosition(rightFlapInitPos);
                leftFlapServo.setPosition(leftFlapInitPos);
            }

            //Collection
            if ((gamepad2.x || gamepad1.x) && !prevX) {
                if (collectionMotor.getPower() == 0) {
                    collectionMotor.setPower(1);
                } else {
                    collectionMotor.setPower(0);
                }
                prevX = true;
            } else if (!(gamepad2.x || gamepad1.x) && prevX) {
                prevX = false;
            }

            //Launcher
            if ((gamepad2.a || gamepad1.a) && !prevA) {
                if (leftLaunchMotor.getPower() == 0) {
                    startLauncher();
                } else {
                    stopLauncher();
                }
                prevA = true;
            } else if (!(gamepad2.a || gamepad1.a) && prevA) {
                prevA = false;
            }

            //Launch
            if ((gamepad2.b || gamepad1.b) && leftLaunchMotor.getPower() != 0) {
                liftTime.reset();
                liftToLaunch();
            }
            if (liftTime.time() >= 300) {
                leftLiftServo.setPosition(leftLiftInitPos);
                rightLiftServo.setPosition(rightLiftInitPos);
            }

//            Regular Launcher Stuff
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                pos = launcherServoShortPos;
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                pos = launcherServoFarPos;
            }

//            Launcher Testing
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (pos + 0.002 <= 1)
                    pos += 0.002;
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (pos - 0.002 >= 0)
                    pos -= 0.002;
            }

            launcherServo.setPosition(pos);
            logData("position", pos);

//            RPM log
            if (launchTestingTimer.time() >= launchTestingWaitTime) {
                launchTestingRpm = (getCurrentRpm(launcherEncoderPpr, leftLaunchMotor, launchTestingWaitTime) + getCurrentRpm(launcherEncoderPpr, rightLaunchMotor, launchTestingWaitTime)) / 2;
                launchTestingTimer.reset();
                encoderStartPos.put(leftLaunchMotor, Math.abs(leftLaunchMotor.getCurrentPosition()));
                encoderStartPos.put(rightLaunchMotor, Math.abs(rightLaunchMotor.getCurrentPosition()));
            }
            logData("launcher rpm", launchTestingRpm);

            updateTelemetry();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void goDirection(double magnitude, double angle) throws InterruptedException {
        if (angleIsNearAngle(angle, RIGHT)) {
            goRight(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD_RIGHT)) {
            goDiagonalForwardRight(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD)) {
            goForward(magnitude);
        } else if (angleIsNearAngle(angle, FORWARD_LEFT)) {
            goDiagonalForwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, LEFT)) {
            goLeft(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD_LEFT)) {
            goDiagonalBackwardLeft(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD)) {
            goBackward(magnitude);
        } else if (angleIsNearAngle(angle, BACKWARD_RIGHT)) {
            goDiagonalBackwardRight(magnitude);
        } else {
            stopRobot();
            logData("ERROR!!!", "ERROR!!!");
        }
    }

    public boolean angleIsNearAngle(double angle1, double angle2) {
        while (angle1 >= 360) {
            angle1 -= 360;
        }
        while (angle1 < 0) {
            angle1 += 360;
        }
        while (angle2 >= 360) {
            angle2 -= 360;
        }
        while (angle2 < 0) {
            angle2 += 360;
        }
        double diff = Math.abs(angle2 - angle1);
        return diff < 45.0 / 2 || diff > 360 - 45.0 / 2;
    }
}
