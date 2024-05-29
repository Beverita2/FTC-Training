package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.control;
public interface Drivetrain {
    void drive(double axial, double lateral, double yaw);
    void drive(double leftFront, double rightFront, double leftBack, double rightBack);
}
