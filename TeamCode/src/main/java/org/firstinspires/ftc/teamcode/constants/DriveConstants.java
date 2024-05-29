package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    public static final double DRIVETRAIN_TICKS_PER_REV = 384.5;
    public static final double DRIVETRAIN_MAX_RPM = 435;
    public static final boolean RUN_USING_ENCODER = false;
    public static final double WINCH_PULLEY_CIRCUMFERENCE = 11.2;
    public static final double ELEVATOR_TICKS_PER_REV = 537.6;

    public static final double ELEVATOR_MAX_RPM = 312;
    public static final double ELEVATOR_MAX_TICKS_PER_SECOND = ELEVATOR_MAX_RPM * ELEVATOR_TICKS_PER_REV / 60;
    public static final double GATE_OPEN_POSITION  = 1;
    public static final double GATE_CLOSE_POSITION = 0;
    public static final double ARM_UP_POSITION  = 1;
    public static final double ARM_DOWN_POSITION = 0;
    public static final double BOX_BACKBOARD_POSITION  = 1;
    public static final double BOX_GROUND_POSITION = 0;
    public static final double INTAKE_UP_POSITION  = 1;
    public static final double INTAKE_DOWN_POSITION = 0;
    public static final double PLANE_LAUNCH_POSITION = 1;
    public static final double PLANE_INIT_POSITION = 0;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(25, 0, 4,
            12.672978548719577);

    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = 1.01; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.79; // in
    public static double kV = 1.0 / rpmToVelocity(DRIVETRAIN_MAX_RPM);
    ;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double MAX_VEL = 55.0632145392716184;
    public static double MAX_ACCEL = 55.0632145392716184;
    public static double MAX_ANG_VEL = 4.7961889775488435;
    public static double MAX_ANG_ACCEL = 4.7961889775488435;
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);
    public static double LATERAL_MULTIPLIER = 1.192;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static double drivetrainTicksPerInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / DRIVETRAIN_TICKS_PER_REV;
    }

    public static double elevatorCmToTicks(double cm) {
        return ELEVATOR_TICKS_PER_REV * cm / WINCH_PULLEY_CIRCUMFERENCE;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}