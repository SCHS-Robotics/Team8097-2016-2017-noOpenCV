package org.firstinspires.ftc.teamcode;

public abstract class CompetitionAutonomous extends Autonomous {

    public void shoot() throws InterruptedException {
        int numTries = numParticles();
        startLauncher();
        sleep(1500);
        for (int i = 0; i < numTries; i++) {
            liftToLaunch();
            sleep(300);
            leftLiftServo.setPosition(leftLiftInitPos);
            rightLiftServo.setPosition(rightLiftInitPos);
            if (i < numTries - 1) {
                sleep(1500);
            }
        }
        stopLauncher();
    }

    public abstract int numParticles();
}