package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Shoot 1 Autonomous", group = "OpMode")
public class RedShootAutonomous extends RedAutonomousOpMode {
    @Override
    public boolean shouldShoot() {
        return true;
    }

    @Override
    public int numParticles() {
        return 1;
    }
}
