package org.firstinspires.ftc.teamcode.control.limbs;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.control.Claws;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ClawsController implements Claws {
    private final RobotHardware robotHardware;
    private int clawState = 1;
    private boolean armState = true;
    private boolean planeLauncherState = false;

    public ClawsController(RobotHardware robotHardware) {
        this.robotHardware = robotHardware;
    }

    @Override
    public boolean useClaws() {
        return true;
    }

    @Override
    public void useClaws(boolean state) {
    }

    @Override
    public boolean useArm(){
        return armState;
    }

    @Override
    public void useArm(boolean state){
    }

    @Override
    public void usePlaneLauncher(){
        if(planeLauncherState){
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_LAUNCH_POSITION);
            planeLauncherState = false;
        }else{
            robotHardware.getPlaneLauncherServo().setPosition(DriveConstants.PLANE_INIT_POSITION);
            planeLauncherState = true;
        }
    }

    @Override
    public boolean isClaws() {
        return true;
    }
}
