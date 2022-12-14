package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TowerController
{

    //setup variables, motors, and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor uBar;
    private int uBarLevel;
    private double uBarPower;

    private DcMotor screw;
    private int screwLevel;
    private TouchSensor highSensor;
    private TouchSensor lowSensor;
    private int previousLevel;
    private int level = 1;

//    private Servo intake2;
    private DcMotor intake;
//    private int intakeLevel;
//    private TouchSensor intakeSensor;
    public int intakePick = 0;

    public TowerController (HardwareMap hardwareMap)
    {

        //Setup motors
        highSensor = hardwareMap.get(TouchSensor.class, "highSensor");
        lowSensor = hardwareMap.get(TouchSensor.class, "lowSensor");
        screw = hardwareMap.get(DcMotor.class, "screw");
        uBar = hardwareMap.get(DcMotor.class, "uBar");

//        intake2 = hardwareMap.get(Servo.class, "intake2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        screw.setDirection(DcMotor.Direction.FORWARD);
        uBar.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Commit out if no intake sensor
//        intakeSensor = hardwareMap.get(TouchSensor.class, "intakeSensor");


        //setup encoder
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        screwLevel = 1000;
//        screw.setDirection(DcMotor.Direction.FORWARD);
//        screw.setTargetPosition(screwLevel);
//        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        screw.setPower(-0.3);
//
//        while ((screw.isBusy() && (screw.getCurrentPosition() <= 1000)) || (screw.isBusy() && (lowSensor.isPressed())))
//        {
//            if (lowSensor.isPressed())
//            {
//                break;
//            }
//        }
//
//        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        uBar.setPower(0);
//        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        screwLevel = 0;
    }

    /**
     *
     * @param uBarTarget
     * @param speed
     * @param telemetry
     */
    private void driveUBarUp(double uBarTarget, double speed, Telemetry telemetry)
    {
        uBarLevel -= uBarTarget;
        uBar.setTargetPosition(uBarLevel);
//        uBar.setDirection(DcMotor.Direction.REVERSE);
        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uBar.setPower(speed);

        while (uBar.isBusy() && ((/*-1 */ uBar.getCurrentPosition()) <= uBarTarget))
        //while (uBar.isBusy())
        {
            telemetry.addData("UBar ticks = ", "%d", uBar.getCurrentPosition());
            telemetry.update();
            uBar.setPower(speed);
        }

        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBarLevel = 0;
    }

    private void driveUBarDown(double uBarTarget, double speed, Telemetry telemetry)
    {
        uBarLevel += uBarTarget;
        uBar.setTargetPosition(uBarLevel);
//        uBar.setDirection(DcMotor.Direction.FORWARD);
        uBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uBar.setPower(-speed);

        while (uBar.isBusy() && ((-1 * uBar.getCurrentPosition()) <= uBarTarget))
        {
            telemetry.addData("UBar ticks = ", "%d", -1 * uBar.getCurrentPosition());
            telemetry.update();
            uBar.setPower(speed);
        }
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
        uBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uBarLevel = 0;
    }

    public void uBarUp(Gamepad gamepad)
    {
        while (gamepad.y)
        {
            uBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uBar.setPower(1);
        }
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
    }

    public void uBarDown(Gamepad gamepad)
    {
        while (gamepad.a)
        {
            uBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uBar.setPower(-1);
        }
        uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uBar.setPower(0);
    }

    private void driveScrewUp(double screwTarget, double speed, Telemetry telemetry)
    {
        // Sets target position
        screwLevel -= screwTarget;
        // Sets direction
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        // sets run mode
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();
        // sets power
        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            // shows how many ticks
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();

            // Stops if sensor is true
            if (highSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();
        }

        // breaks the motor
        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

    private void driveScrewDown(double screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(-speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (lowSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        screwLevel = 0;
    }

    public void handleIntake (double intakeTarget, double speed, Telemetry telemetry)
    {
//        if(intakePick == 1)
//        {
//            intake2.setPosition(1);
//            intake2.getPosition();
////            while (intake2.getPosition() == 0.5){}
//            intakePick = 1954;
//        }
//        else  if (intakePick == 0)
//        {
//            intake2.setPosition(0);
//            intake2.getPosition();
////            while (intake2.getPosition() == 0){}
//            intakeTarget = 1954;
//        }

        // Wheels
//        if(intakePick)
//        {
//            intakeLevel -= intakeTarget;
//            intake.setDirection(DcMotor.Direction.FORWARD);
//            intake.setTargetPosition(screwLevel);
//            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            screw.setPower(speed);
//            while ((intake.isBusy() && (intake.getCurrentPosition() <= intakeTarget)))
//            {
//                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//                telemetry.update();
//            }
//
//            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            screw.setPower(0);
//            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            screwLevel = 0;
//        }
//
//        if (!intakePick)
//        {
//            intakeLevel -= intakeTarget;
//            intake.setDirection(DcMotor.Direction.REVERSE);
//            intake.setTargetPosition(screwLevel);
//            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            screw.setPower(speed);
//            while ((intake.isBusy() && (intake.getCurrentPosition() <= intakeTarget)))
//            {
//                telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//                telemetry.update();
//            }
//
//            screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            screw.setPower(0);
//            screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            screwLevel = 0;
//        }
    }

    public void screwDriveToLevelUp(int screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.FORWARD);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (highSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (lowSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void screwDriveToLevelDown(int screwTarget, double speed, Telemetry telemetry)
    {
        screwLevel -= screwTarget;
        screw.setDirection(DcMotor.Direction.REVERSE);
        screw.setTargetPosition(screwLevel);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//        telemetry.update();

        screw.setPower(-speed);

        while ((screw.isBusy() && (screw.getCurrentPosition() <= screwTarget)) || (screw.isBusy() && (lowSensor.isPressed())))
        {
            telemetry.addData("Screw ticks = ", "%d", screw.getCurrentPosition());
//            telemetry.update();


            if (lowSensor.isPressed())
            {
                telemetry.addData("highSensor is pressed", "");
                telemetry.update();
                break;
            }
            telemetry.update();

        }

        screw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void switchLevel(Telemetry telemetry)
    {
        switch (level)
        {
            case 1:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 0, 1, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(0 - screwLevel, 1, telemetry);
                }
                break;

            case 2:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 1000, 1, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(1000 - screwLevel, 1, telemetry);
                }
                break;

            case 3:
                if (level < previousLevel)
                {
                    screwDriveToLevelDown(screwLevel - 2000, 1, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(2000 - screwLevel, 1, telemetry);
                }
                break;

            case 4:
                if (level < previousLevel)
                {

                    screwDriveToLevelDown(screwLevel - 3000, 1, telemetry);
                }
                else
                {
                    screwDriveToLevelUp(3000 - screwLevel, 1, telemetry);
                }
                break;

            default:
                break;
        }
    }

    public void handleGamepad(Gamepad gamepad, Telemetry telemetry)
    {
          // Intake sensor prints message on whether or not the intake has a cone
//        if (intakeSensor.isPressed())
//        {
//            telemetry.addData("Intake Sensor", "&s", "Pressed");
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addData("Intake Sensor", "&s", "NOT Pressed");
//            telemetry.update();
//        }

        //Screw spin
        if(gamepad.dpad_up)
        {
            driveScrewUp(2000, 100, telemetry);
        }
        if(gamepad.dpad_down)
        {
            driveScrewDown(2000, 100, telemetry);
        }

        // for go to level
//        if (gamepad.dpad_up && level < 4)
//        {
//            previousLevel = level;
//            level++;
//            switchLevel(telemetry);
//        }
//        if (gamepad.dpad_down && level > 1)
//        {
//            previousLevel = level;
//            level--;
//            switchLevel(telemetry);
//        }

        //U Bar
//        if(gamepad.b && !gamepad.start)
//        {
//            gamepad.b = false;
//            driveUBarUp(330.06875, 0.3, telemetry);
//            //60 degrees 330.06875
//        }
//        if(gamepad.a)
//        {
//            gamepad.a = false;
//            driveUBarUp(165.034375, 0.3, telemetry);
//            //30 degrees 165.034375
//        }
//        if(gamepad.x)
//        {
//            gamepad.x = false;
//            driveUBarDown(5000, 0.3, telemetry);
//            //60 degrees
//        }
//        if(gamepad.y)
//        {
//            gamepad.y = false;
//            driveUBarDown(5000, 0.3, telemetry);
//            //30 degrees
//        }

//        if (gamepad.y)
//        {
//            uBarUp(gamepad);
//        }
//        else if (gamepad.a)
//        {
//            uBarDown(gamepad);
//        }
//        else
//        {
//            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }

        double uBarPower;

        double drive = -gamepad.left_stick_y;
        uBarPower = Range.clip(drive, -1, 1);
        uBar.setPower(uBarPower);

        if (/*gamepad.left_stick_y == 0*/ uBarPower == 0 /* drive == 0 */)
        {
            telemetry.addData( "Breaking", "", "");
            telemetry.update();
            uBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //intake wheel
        if (gamepad.left_bumper)
        {
            intake.setPower(1);
        }
        else if (gamepad.right_bumper)
        {
            intake.setPower(-1);
        }
        else
        {
            intake.setPower(0);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        //Intake scissor cone
//        if(gamepad.right_bumper)
//        {
//            intakePick = 1;
//            handleIntake(50, 1, telemetry);
//        }
//        if(gamepad.left_bumper)
//        {
//            intakePick = 0;
//            handleIntake(50, 1, telemetry);
//        }

    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    //hi. you found me. -SECRET COMMENT