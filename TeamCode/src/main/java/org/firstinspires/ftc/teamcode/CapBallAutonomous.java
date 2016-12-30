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

        goForwardDistance(DEFAULT_FORWARD_SPEED, 25);
        shoot();
        sleep(10000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 150);
        sleep(2000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 12);


        while (opModeIsActive()) {
            idle();
        }
    }
}
