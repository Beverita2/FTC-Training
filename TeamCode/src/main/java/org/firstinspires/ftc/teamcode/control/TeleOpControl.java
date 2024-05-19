package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import lombok.Getter;
import lombok.Setter;

public abstract class

TeleOpControl extends LinearOpMode implements Drivetrain {
    protected GamepadEx mainGamepad;
    protected GamepadEx secondaryGamepad;
    protected boolean invertedGamepads = false;
    protected boolean skipInit = true;
    protected boolean skipStart = true;

    protected final RobotHardware robotHardware = new RobotHardware(this);

    @Override
    public final void runOpMode() throws InterruptedException {
        overrideSettings();
        robotHardware.initTeleOp();
        mainGamepad = new GamepadEx(gamepad1);
        secondaryGamepad = new GamepadEx(gamepad2);
        if (invertedGamepads) {
            GamepadEx aux = secondaryGamepad;
            secondaryGamepad = mainGamepad;
            mainGamepad = aux;
        }

        if (skipInit) waitForStart();
        else {
            enhanced_init();
            telemetry.update();
            while (opModeInInit()) {
                enhanced_init_loop();
                telemetry.update();
            }
        }

        if (!skipStart) {
            if (!isStopRequested()) enhanced_start();
        }

        while (opModeIsActive()) {
            run();

            if (mainGamepad.wasJustReleased(GamepadKeys.Button.Y)) {
                debugMainGamepad();
            }

            if (secondaryGamepad.wasJustReleased(GamepadKeys.Button.Y)) {
                debugSecondaryGamepad();
            }

            telemetry.update();
            mainGamepad.readButtons();
            secondaryGamepad.readButtons();
        }

        if (isStopRequested()) {
            enhanced_stop();
        }
    }

    protected void overrideSettings() {

    }

    protected void debugMainGamepad() {

    }

    protected void debugSecondaryGamepad() {

    }

    protected void enhanced_init() {

    }

    protected void enhanced_init_loop() {

    }

    protected void enhanced_stop() {

    }

    protected void enhanced_start() {

    }

    protected abstract void run();

    @Override
    public void drive(double axial, double lateral, double yaw) {
        double targetLeftFrontPower = axial + lateral + yaw;
        double targetRightFrontPower = axial - lateral - yaw;
        double targetLeftBackPower = axial - lateral + yaw;
        double targetRightBackPower = axial + lateral - yaw;

        double max = Math.max(Math.abs(targetLeftFrontPower), Math.abs(targetRightFrontPower));
        max = Math.max(max, Math.abs(targetLeftBackPower));
        max = Math.max(max, Math.abs(targetRightBackPower));

        if (max > 1) {
            targetLeftFrontPower /= max;
            targetRightFrontPower /= max;
            targetLeftBackPower /= max;
            targetRightBackPower /= max;
        }

        targetRightBackPower *= powerScale;
        targetRightFrontPower *= powerScale;
        targetLeftFrontPower *= powerScale;
        targetLeftBackPower *= powerScale;

        robotHardware.getLeftFrontMotor().setPower(targetLeftFrontPower);
        robotHardware.getRightFrontMotor().setPower(targetRightFrontPower);
        robotHardware.getLeftBackMotor().setPower(targetLeftBackPower);
        robotHardware.getRightBackMotor().setPower(targetRightBackPower);
    }


    @Override
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack) {
        robotHardware.getLeftFrontMotor().setPower(leftFront);
        robotHardware.getRightFrontMotor().setPower(rightFront);
        robotHardware.getLeftBackMotor().setPower(leftBack);
        robotHardware.getRightBackMotor().setPower(rightBack);
    }

    @Setter
    @Getter
    private double powerScale = 1;
}
