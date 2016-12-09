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

        //TODO basically wait 10 seconds, goForward to hit cap ball, pause for a second, go a little bit forward to partially park

        while (opModeIsActive()) {
            idle();
        }
    }
}
