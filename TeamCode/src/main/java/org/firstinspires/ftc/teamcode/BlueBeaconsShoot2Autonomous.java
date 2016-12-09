package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Beacons Shoot 2 Autonomous", group = "OpMode")
public class BlueBeaconsShoot2Autonomous extends BlueBeaconsShootAutonomous {
    @Override
    public int numParticles() {
        return 2;
    }
}
