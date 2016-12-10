package org.firstinspires.ftc.teamcode;

public abstract class CompetitionAutonomous extends Autonomous {

    public void shoot() throws InterruptedException {
        int numTries = numParticles() + 1;
        startLauncher();
        sleep(500);
        for (int i = 0; i < numTries + 1; i++) {
            leftLiftServo.setPosition(leftLiftEndPos);
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