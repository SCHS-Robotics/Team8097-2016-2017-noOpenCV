package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot 1 Autonomous", group = "OpMode")
public class ShootAutonomous extends CompetitionAutonomous {
    @Override
    public int numParticles() {
        return 1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        logData("Status", "Initialized");
        updateTelemetry();

        allInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        shoot();

        while (opModeIsActive()) {
            idle();
        }
    }
}
