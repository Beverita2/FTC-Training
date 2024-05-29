package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.control;

import lombok.Getter;
public interface Elevator {

    enum ElevatorLevel {
        BASE(0),
        MID(25),
        HIGH(50),
        MAX(75);

        @Getter
        private double height = 0;

        ElevatorLevel(double height) {
            this.height = height;
        }
    }
    void update();

    void setTarget(int target);

    void setTarget(ElevatorLevel level);

    int getTarget();

    int getCurrentPosition();

    boolean isBypass();

    void setBypass(boolean bypass);
}
