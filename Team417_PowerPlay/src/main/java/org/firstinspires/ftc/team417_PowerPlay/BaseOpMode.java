package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;

abstract public class BaseOpMode extends LinearOpMode {

    DcMotor motorArm;

    Servo grabberServo;
    Toggler grabberToggle;

    SampleMecanumDrive drive;
    public static final double GRABBER_OPEN = 0.9;
    public static final double GRABBER_CLOSED = 0.4;
    public static final double GRABBER_HALF_CLOSED = 0.8;

    public static final int MAX_ARM_POSITION = -1500;
    public static final int GRD_JUNCT_ARM_POSITION = -400;
    public static final int LOW_JUNCT_ARM_POSITION = -1000;
    public static final int MID_JUNCT_ARM_POSITION = -1500;
    public static final int MIN_ARM_POSITION = 0;

    public void initializeHardware() {
        drive = new SampleMecanumDrive(hardwareMap);

        motorArm = hardwareMap.dcMotor.get("motorArm");

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberServo = hardwareMap.servo.get("grabberServo");
        grabberToggle = new Toggler();

        drive.setMotorPowers(0, 0, 0, 0);
        motorArm.setPower(0);
    }

    public void mecanumDrive(double x, double y, double turning) {
        double powerFL;
        double powerFR;
        double powerBL;
        double powerBR;

        powerFL = y + x + turning;
        powerFR = y - x - turning;
        powerBL = y - x + turning;
        powerBR = y + x - turning;

        drive.setMotorPowers(powerFL, powerBL, powerBR, powerFR);
    }
}
