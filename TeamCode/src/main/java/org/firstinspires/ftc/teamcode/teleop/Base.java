package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.TeleOpControl;

@TeleOp(name = "Main", group = "Movement")
public class Base extends TeleOpControl {

    private static final double ELEVATOR_SCALE = 0.25;

    @Override
    protected void debugSecondaryGamepad() {
        if (!robotHardware.getElevatorController().isBypass()) {
            robotHardware.getElevatorController().setBypass(true);
        } else {
            robotHardware.getElevatorController().setBypass(false);
            robotHardware.getRightElevatorMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotHardware.getLeftElevatorMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robotHardware.getRightElevatorMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robotHardware.getLeftElevatorMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robotHardware.getElevatorController().setTarget(0);
        }
    }

    @Override
    protected void run() {
        double axial = mainGamepad.getLeftY();
        double lateral = mainGamepad.getLeftX();
        double yaw = mainGamepad.getRightX();

        drive(axial, lateral, yaw);

        if (mainGamepad.wasJustReleased(GamepadKeys.Button.A)) {
            setPowerScale(1.25 - getPowerScale());
        }

        double intakePower = secondaryGamepad.getLeftY() * 0.8;
        robotHardware.getIntake().setPower(intakePower);


        double elevatorTarget = robotHardware.getElevatorController().getTarget() + DriveConstants.elevatorCmToTicks(secondaryGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - secondaryGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * ELEVATOR_SCALE;
        robotHardware.getElevatorController().setTarget((int) elevatorTarget);

        robotHardware.getElevatorController().update();
        robotHardware.getMecanumDriveController().update();
    }
}
