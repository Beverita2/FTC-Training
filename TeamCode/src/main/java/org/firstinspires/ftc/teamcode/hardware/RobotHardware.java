package org.firstinspires.ftc.teamcode.hardware;

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
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.Elevator;
import org.firstinspires.ftc.teamcode.control.MecanumDriveController;

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
    private Servo unnamedServo1 = null;
    @Getter
    private Servo unnamedServo2 = null;
    @Getter
    private Servo unnamedServo3 = null;
    @Getter
    private Servo unnamedServo4 = null;
    @Getter
    private Servo unnamedServo5 = null;
    @Getter
    private Servo unnamedServo6 = null;
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

    public void initLynxModule() {
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : opMode.hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initMecanumDriveController() {
        initDrivetrainMotors();
        initIMU();
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

        setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DriveConstants.MOTOR_VELO_PID);

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setAllBrake();

        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void initClaws() {

        unnamedServo1 = opMode.hardwareMap.get(Servo.class, "unnamedServo1");
        unnamedServo2 = opMode.hardwareMap.get(Servo.class, "unnamedServo2");
        unnamedServo3 = opMode.hardwareMap.get(Servo.class, "unnamedServo3");
        unnamedServo4 = opMode.hardwareMap.get(Servo.class, "unnamedServo4");
        unnamedServo5 = opMode.hardwareMap.get(Servo.class, "unnamedServo5");
        unnamedServo6 = opMode.hardwareMap.get(Servo.class, "unnamedServo6");
        planeLauncherServo = opMode.hardwareMap.get(Servo.class, "plane_launcher");

        planeLauncherServo.scaleRange(0,1);
        unnamedServo1.scaleRange(0,1);
        unnamedServo2.scaleRange(0,1);
        unnamedServo3.scaleRange(0,1);
        unnamedServo4.scaleRange(0,1);
        unnamedServo5.scaleRange(0,1);
        unnamedServo6.scaleRange(0,1);
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
        //initWebcam();
    }

    public void initTeleOp() {
        initDrivetrainMotors();
    }


    public void initWebcam() {
        webcam = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

}
