package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Beacons Shoot 2 Autonomous", group = "OpMode")
public class RedBeaconsShoot2Autonomous extends RedBeaconsShootAutonomous {
    @Override
    public int numParticles() {
        return 2;
    }
}
