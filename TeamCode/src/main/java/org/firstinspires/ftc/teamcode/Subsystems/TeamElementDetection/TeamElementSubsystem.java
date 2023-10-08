package org.firstinspires.ftc.teamcode.Subsystems.TeamElementDetection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Subsystems.TeamElementDetection.Pipeline.SplitAveragePipeline;

public class TeamElementSubsystem {
    OpenCvCamera camera;
    SplitAveragePipeline splitAveragePipeline;
    int camW = 800;
    int camH = 448;

    int zone = 1;

    public TeamElementSubsystem(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        splitAveragePipeline = new SplitAveragePipeline();

        camera.setPipeline(splitAveragePipeline);
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
    }

    public int ElementDetection(Telemetry telemetry) {
        zone = SplitAveragePipeline.get_element_zone();
        telemetry.addData("Element in zone", zone);
        return zone;
    }
}

