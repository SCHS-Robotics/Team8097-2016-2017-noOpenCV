package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Beacons Shoot 1 Autonomous", group = "OpMode")
public class RedBeaconsShootAutonomous extends RedBeaconsAutonomous {
    @Override
    public boolean shouldShoot() {
        return true;
    }

    @Override
    public int numParticles() {
        return 1;
    }
}
