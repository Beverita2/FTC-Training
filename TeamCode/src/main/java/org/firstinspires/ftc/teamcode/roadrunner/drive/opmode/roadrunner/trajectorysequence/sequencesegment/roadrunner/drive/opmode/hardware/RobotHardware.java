package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.hardware;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.Elevator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.MecanumDriveController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.limbs.ElevatorController;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.limbs.ServoController;

import java.util.ArrayList;
import java.util.List;

import lombok.Getter;

public class RobotHardware {

    private final OpMode opMode;

    public RobotHardware(OpMode opMode) {
        this.opMode = opMode;
    }

    @Getter
    private DcMotorEx leftFrontMotor = null;
    @Getter
    private DcMotorEx leftBackMotor = null;
    @Getter
    private DcMotorEx rightFrontMotor = null;
    @Getter
    private DcMotorEx rightBackMotor = null;
    @Getter
    private DcMotorEx leftElevatorMotor = null;
    @Getter
    private DcMotorEx rightElevatorMotor = null;
    @Getter
    private DcMotorEx intake = null;
    @Getter
    private Servo planeLauncherServo = null;
    @Getter
    private Servo armServo = null;
    @Getter
    private Servo boxServo = null;
    @Getter
    private Servo gateServo = null;
    @Getter
    private Servo intakeServo = null;
    @Getter
    private DcMotorEx perpendicular = null;
    @Getter
    private DcMotorEx parallel = null;
    @Getter
    private Rev2mDistanceSensor sensor1 = null;
    @Getter
    private Rev2mDistanceSensor sensor2 = null;
    @Getter
    private Rev2mDistanceSensor sensor3 = null;
    @Getter
    private WebcamName webcam = null;
    @Getter
    private BHI260IMU imu = null;

    @Getter
    private final List<DcMotorEx> drivetrainMotors = new ArrayList<>();
    @Getter
    private VoltageSensor batteryVoltageSensor = null;
    @Getter
    private MecanumDriveController mecanumDriveController = null;
    @Getter
    private Elevator elevatorController = null;
    @Getter
    private ServoController servoController = null;

    public void initLynxModule() {
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initMecanumDriveController() {
        initDrivetrainMotors();
        initServos();
        mecanumDriveController = new MecanumDriveController(this);
    }

    public void initDrivetrainMotors() {
        initLynxModule();

        leftFrontMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_back");
        leftElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, "left_elevator");
        rightElevatorMotor = opMode.hardwareMap.get(DcMotorEx.class, "right_elevator");
        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        parallel = opMode.hardwareMap.get(DcMotorEx.class, "left_front");
        perpendicular = opMode.hardwareMap.get(DcMotorEx.class, "left_back");

        drivetrainMotors.add(leftFrontMotor);
        drivetrainMotors.add(leftBackMotor);
        drivetrainMotors.add(rightBackMotor);
        drivetrainMotors.add(rightFrontMotor);

        for (DcMotorEx motor : drivetrainMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftElevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setAllMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setAllMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID);

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setAllBrake();

        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevatorController = new ElevatorController(this);
        servoController = new ServoController(this);
    }

    public void setAllMode(DcMotor.RunMode runMode) {
        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setMode(runMode));
    }

    public void setAllBrake() {
        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setAllPowers(double v, double v1, double v2, double v3) {
        leftFrontMotor.setPower(v);
        leftBackMotor.setPower(v1);
        rightBackMotor.setPower(v2);
        rightFrontMotor.setPower(v3);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        drivetrainMotors.forEach(dcMotorEx -> dcMotorEx.setPIDFCoefficients(runMode, compensatedCoefficients));
    }

    public void initServos() {

        armServo = opMode.hardwareMap.get(Servo.class, "arm");
        boxServo = opMode.hardwareMap.get(Servo.class, "box");
        gateServo = opMode.hardwareMap.get(Servo.class, "gate");
        intakeServo = opMode.hardwareMap.get(Servo.class, "intake");
        planeLauncherServo = opMode.hardwareMap.get(Servo.class, "plane_launcher");

        armServo.scaleRange(0,1);
        boxServo.scaleRange(0,1);
        gateServo.scaleRange(0,1);
        intakeServo.scaleRange(0,1);
        planeLauncherServo.scaleRange(0,1);
    }

    public void initSensors() {
        sensor1 = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "sensor2");
        sensor3 = (Rev2mDistanceSensor) opMode.hardwareMap.get(DistanceSensor.class, "sensor3");
    }

    public void initIMU() {
        imu = opMode.hardwareMap.get(BHI260IMU.class, "imu");
    }

    public void initAutonomous() {
        initMecanumDriveController();
        initWebcam();
        initIMU();
    }

    public void initTeleOp() {
        initMecanumDriveController();
    }


    public void initWebcam() {
        webcam = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

}
