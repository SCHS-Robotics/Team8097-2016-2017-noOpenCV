package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Shoot 2 Autonomous", group = "OpMode")
public class Shoot2Autonomous extends ShootAutonomous {
    @Override
    public int numParticles() {
        return 2;
    }
}
