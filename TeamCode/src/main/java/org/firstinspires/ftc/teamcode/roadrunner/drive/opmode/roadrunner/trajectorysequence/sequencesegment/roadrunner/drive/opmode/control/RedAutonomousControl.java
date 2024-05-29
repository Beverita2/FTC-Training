package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.constants.RedPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.roadrunner.trajectorysequence.sequencesegment.roadrunner.drive.opmode.hardware.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import lombok.Getter;

public abstract class RedAutonomousControl extends LinearOpMode {

    public OpenCvCamera camera;
    public RedPipeline splitAveragePipeline;
    public int camW = 1280;
    public int camH = 720;
    public int zone = 1;
    public int elementZone = 1;

    @Getter
    protected final RobotHardware robotHardware = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware.initAutonomous();

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new RedPipeline();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        while (opModeInInit()){
            elementZone = elementDetection(telemetry);
            telemetry.addData("getMaxDistance", getMaxDistance());
            telemetry.addData("Zone", elementZone);
            telemetry.update();
        }

        initTrajectories();

        if (isStopRequested()) return;

        run();
    }

    protected abstract void initTrajectories();

    protected abstract void run();

    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

    public double getMaxDistance(){
        return splitAveragePipeline.getMaxDistance();
    }
}

