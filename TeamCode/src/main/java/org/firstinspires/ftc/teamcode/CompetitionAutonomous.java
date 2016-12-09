package org.firstinspires.ftc.teamcode;

public abstract class CompetitionAutonomous extends Autonomous {

    public void shoot() throws InterruptedException {
        startLauncher();
        sleep(500);
        for (int i = 0; i < numParticles(); i++) {
            leftLiftServo.setPosition(leftLiftEndPos);
            rightLiftServo.setPosition(rightLiftEndPos);
            sleep(500);
            leftLiftServo.setPosition(leftLiftInitPos);
            rightLiftServo.setPosition(rightLiftInitPos);
            if (i < numParticles() - 1) {
                sleep(500);
            }
        }
        stopLauncher();
    }

    public abstract int numParticles();
}