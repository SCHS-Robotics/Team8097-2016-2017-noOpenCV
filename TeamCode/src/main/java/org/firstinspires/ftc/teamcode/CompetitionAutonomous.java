package org.firstinspires.ftc.teamcode;

public abstract class CompetitionAutonomous extends Autonomous {

    public void shoot() throws InterruptedException {
        int numTries = numParticles() + 1;
        startLauncher();
        sleep(1000);
        for (int i = 0; i < numTries + 1; i++) {
            leftLiftServo.setPosition(leftLiftEndPos);
            sleep(23);
            rightLiftServo.setPosition(rightLiftEndPos);
            sleep(300);
            leftLiftServo.setPosition(leftLiftInitPos);
            rightLiftServo.setPosition(rightLiftInitPos);
            if (i < numTries - 1) {
                sleep(1000);
            }
        }
        stopLauncher();
    }

    public abstract int numParticles();
}