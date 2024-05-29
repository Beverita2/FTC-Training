package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.Elevator;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control.TeleOpControl;

@TeleOp(name = "Main", group = "Movement")
public class Main extends TeleOpControl {

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
            setPowerScale(1 - getPowerScale());
        }

        if(mainGamepad.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
            robotHardware.getServoController().useGate();
        }

        if(secondaryGamepad.wasJustPressed(GamepadKeys.Button.A)){
            robotHardware.getServoController().useIntake();
            robotHardware.getServoController().useGate();
        }

        if(secondaryGamepad.wasJustReleased(GamepadKeys.Button.A)){
            robotHardware.getServoController().useIntake();
            robotHardware.getServoController().useGate();
        }

        if(secondaryGamepad.getButton(GamepadKeys.Button.A)){
            robotHardware.getIntake().setPower(0.5);
        }else{
            robotHardware.getIntake().setPower(0);
        }

        if(secondaryGamepad.wasJustReleased(GamepadKeys.Button.B)){
            robotHardware.getServoController().useOutTake();
        }

        if(secondaryGamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
            robotHardware.getServoController().useOutTake();
            robotHardware.getElevatorController().setTarget(Elevator.ElevatorLevel.BASE);
        }


        if(secondaryGamepad.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)){
            robotHardware.getServoController().useOutTake();
            robotHardware.getElevatorController().setTarget(Elevator.ElevatorLevel.MID);
        }


        if(secondaryGamepad.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)){
            robotHardware.getServoController().useOutTake();
            robotHardware.getElevatorController().setTarget(Elevator.ElevatorLevel.HIGH);
        }


        double elevatorTarget = robotHardware.getElevatorController().getTarget() + DriveConstants.elevatorCmToTicks(secondaryGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - secondaryGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * ELEVATOR_SCALE;
        robotHardware.getElevatorController().setTarget((int) elevatorTarget);

        robotHardware.getElevatorController().update();
    }
}
