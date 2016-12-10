package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Cap Ball Autonomous", group = "OpMode")
public class CapBallAutonomous extends CompetitionAutonomous {
    @Override
    public int numParticles() {
        return 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(15000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 165);
        sleep(2000);
        goForwardDistance(DEFAULT_FORWARD_SPEED, 12);


        while (opModeIsActive()) {
            idle();
        }
    }
}
