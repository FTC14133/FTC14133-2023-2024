package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class GetPos {

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    public GetPos(HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    public void returnAprilPos(Telemetry telemetry) {

        org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag;
        desiredTag = getDetections(1);
        telemetry.addData("desiredTag", desiredTag);

        if (!(desiredTag == null)) {
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

            telemetry.addData("", "");
            telemetry.addData("Distance from Backdrop", (Math.cos(desiredTag.ftcPose.bearing) * desiredTag.ftcPose.range));
            telemetry.addData("Offset Sideways by", (Math.sin(desiredTag.ftcPose.bearing) * desiredTag.ftcPose.range));
        }

    }


    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection getDetections(int desiredTagID){
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections != null) {
            for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((desiredTagID < 0) || (detection.id == desiredTagID))) {
                    desiredTag = detection;
                    break;
                }
            }
            return desiredTag;
        }
        return null;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTag)
                .build();
    }
}