package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.control.limbs;

import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.control.Servos;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.hardware.RobotHardware;

public class ServoController implements Servos {
    private final RobotHardware robotHardware;
    private boolean planeLauncherState = false;

    private boolean gateState = true;

    private boolean armState = false;

    private boolean intakeState = false;

    public ServoController(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public void useIntake() {
        if(intakeState){
            robotHardware.getIntakeServo().setPosition(DriveConstants.INTAKE_UP_POSITION);
            intakeState = false;
        }else{
            robotHardware.getIntakeServo().setPosition(DriveConstants.INTAKE_DOWN_POSITION);
            intakeState = true;
        }
    }

    @Override
    public void useIntake(boolean state) {
        if(!state){
            robotHardware.getIntakeServo().setPosition(DriveConstants.INTAKE_UP_POSITION);
            intakeState = false;
        }else{
            robotHardware.getIntakeServo().setPosition(DriveConstants.INTAKE_DOWN_POSITION);
            intakeState = true;
        }
    }

    @Override
    public void useGate() {
        if (gateState) {
            robotHardware.getGateServo().setPosition(DriveConstants.GATE_CLOSE_POSITION);
            gateState = false;
        } else {
            robotHardware.getGateServo().setPosition(DriveConstants.GATE_OPEN_POSITION);
            gateState = true;
        }
    }

    @Override
    public void useGate(boolean state) {
        if (!state) {
            robotHardware.getGateServo().setPosition(DriveConstants.GATE_CLOSE_POSITION);
            gateState = false;
        } else {
            robotHardware.getGateServo().setPosition(DriveConstants.GATE_OPEN_POSITION);
            gateState = true;
        }
    }

    @Override
    public void useOutTake() {
        if (armState) {
            robotHardware.getArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            robotHardware.getBoxServo().setPosition(DriveConstants.BOX_GROUND_POSITION);
            armState = false;
        } else {
            robotHardware.getArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            robotHardware.getBoxServo().setPosition(DriveConstants.BOX_BACKBOARD_POSITION);
            armState = true;
        }
    }

    @Override
    public void useOutTake(boolean state) {
        if (!state) {
            robotHardware.getArmServo().setPosition(DriveConstants.ARM_DOWN_POSITION);
            robotHardware.getBoxServo().setPosition(DriveConstants.BOX_GROUND_POSITION);
            armState = false;
        } else {
            robotHardware.getArmServo().setPosition(DriveConstants.ARM_UP_POSITION);
            robotHardware.getBoxServo().setPosition(DriveConstants.BOX_BACKBOARD_POSITION);
            armState = true;
        }
    }

    @Override
    public void usePlaneLauncher() {
        if (planeLauncherState) {
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_INIT_POSITION);
            planeLauncherState = false;
        } else {
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_LAUNCH_POSITION);
            planeLauncherState = true;
        }
    }

    @Override
    public void usePlaneLauncher(boolean state) {
        if (!state) {
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_INIT_POSITION);
            planeLauncherState = false;
        } else {
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_LAUNCH_POSITION);
            planeLauncherState = true;
        }
    }

}
