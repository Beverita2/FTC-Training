package org.firstinspires.ftc.teamcode.control;

import lombok.Getter;
public interface Elevator {

    enum ElevatorLevel {
        BASE(0),
        LOW(37.2),
        MIDDLE(60.8),
        HIGH(88),
        MAX(90);

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
