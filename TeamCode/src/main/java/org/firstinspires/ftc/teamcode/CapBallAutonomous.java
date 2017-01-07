package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Cap Ball Autonomous", group = "OpMode")
public class CapBallAutonomous extends CompetitionAutonomous {
    @Override
    public int numParticles() {
        return 2;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        goBackwardDistance(DEFAULT_FORWARD_SPEED, 60);
        shoot();
        sleep(8000);
        spinRightDegrees(DEFAULT_SPIN_SPEED, 180);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 115);
        sleep(2000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 12);


        while (opModeIsActive()) {
            idle();
        }
    }
}
