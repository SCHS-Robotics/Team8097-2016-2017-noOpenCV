package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Shoot 1 Autonomous", group = "OpMode")
public class BlueShootAutonomous extends BlueAutonomousOpMode {
    @Override
    public boolean shouldShoot() {
        return true;
    }

    @Override
    public int numParticles() {
        return 1;
    }
}
