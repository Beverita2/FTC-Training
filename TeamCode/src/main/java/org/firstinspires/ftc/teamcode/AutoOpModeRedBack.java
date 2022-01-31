// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto OpMode BR", group="16481")
public class AutoOpModeRedBack extends LinearOpMode {

   robot16481 robot = new robot16481();

   @Override
   public void runOpMode() throws InterruptedException {

      robot.telemetry = telemetry;
      robot.hwMap = hardwareMap;
      robot.hardwareSetup();

      robot.initTfod();

      while (!(isStarted() || isStopRequested())) {
         idle();
      }

      while (opModeIsActive()) {

         robot.setClawPosition(robot.CLAW_CLOSE);
         Thread.sleep(1000);
         robot.setArm(robot.elementPosition());
         //robot.armMotor.setPower(0);

         // walk 23 inches
         robot.encoderDrive(23.0, 23.0, 1.0, robot.FORWARD);
         //turn 30n degrees left towards shipping hub
         robot.encoderDrive(8.0, -8.0, 1.0, robot.FORWARD);
         //walk 13 inches toward shipping hub
         robot.encoderDrive(13.0, 13.0, 1.0, robot.FORWARD);
         //open the robot claw
         robot.setClawPosition(robot.CLAW_OPEN);
         Thread.sleep(500);
         //going back 7 inches from shipping hub to allow arm to lower
         robot.encoderDrive(-7.0, -7.0, 0.5, robot.FORWARD);
         //setting the arm to position one after backing out from shipping hub
         robot.setArm(1);
         //walk back to carousel
         //turn back facing the wall
         robot.encoderDrive(-8.0, 8.0, 1.0, robot.FORWARD);
         //go back and hit the wall
         robot.encoderDrive(-22.0, -22.0, 1.0, robot.FORWARD);
         robot.encoderDrive(35, 35, 1.0, robot.STRAFE_RIGHT);


         telemetry.addData("# front left", robot.motorFrontLeft.getPower());
         telemetry.addData("# front right", robot.motorFrontRight.getPower());
         telemetry.addData("# back left", robot.motorBackLeft.getPower());
         telemetry.addData("# back right", robot.motorBackRight.getPower());

         telemetry.addData("pot Value", robot.potMeter.getVoltage());
         telemetry.addData("Servo Value: ", robot.clawServo.getPosition());
         telemetry.update();
         break;
      }
   }
}