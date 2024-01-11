/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;

public class AprilTagDetection {
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    HashMap<Integer, double[]> aprilTagPosit = new HashMap<>();

    final int ROBOT_WIDTH = 17;
    final int ROBOT_HEIGHT = 18;


    public AprilTagDetection(HardwareMap hardwareMap){

        aprilTagPosit.put(1, new double[]{61, 42});
        aprilTagPosit.put(2, new double[]{61, 35});
        aprilTagPosit.put(3, new double[]{61, 28});
        aprilTagPosit.put(4, new double[]{61, -28});
        aprilTagPosit.put(5, new double[]{61, -35});
        aprilTagPosit.put(6, new double[]{61, -42});

        initAprilTag(hardwareMap);
        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    public void calcAprilPos(Telemetry telemetry, SampleMecanumDrive drive) {
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

        desiredTag = getDetections();

        if (desiredTag == null){
            return;
        }

        telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
        telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
        telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
        telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

        telemetry.addData("", "");
        telemetry.addData("Distance from Backdrop", (Math.cos(desiredTag.ftcPose.bearing) * desiredTag.ftcPose.range));
        telemetry.addData("Offset Sideways by", (Math.sin(desiredTag.ftcPose.bearing) * desiredTag.ftcPose.range));


        if (aprilTagPosit.containsKey(desiredTag.id)){

            double bearing = Math.toRadians(desiredTag.ftcPose.bearing);
            double yaw = Math.toRadians(desiredTag.ftcPose.yaw);


            double[] aprilTagPos = aprilTagPosit.get(desiredTag.id);

            double xBackDist = Math.cos(bearing) * desiredTag.ftcPose.range;
            double yBackDist = Math.sin(bearing) * desiredTag.ftcPose.range;
            double[] cameraPos = new double[]{aprilTagPos[0] - xBackDist, aprilTagPos[1] - yBackDist};

            double xRobotTRDist = Math.sin(yaw) * ROBOT_WIDTH;
            double yRobotTRDist = Math.cos(yaw) * ROBOT_WIDTH;
            double[] robotTRPos = new double[]{cameraPos[0] - xRobotTRDist, cameraPos[1] - yRobotTRDist};

            double xRobotBRDist = Math.cos(yaw) * ROBOT_HEIGHT;
            double yRobotBRDist = Math.sin(yaw) * ROBOT_HEIGHT;
            double[] robotBRPos = new double[]{robotTRPos[0] - xRobotBRDist, robotTRPos[1] - yRobotBRDist};

            double[] robotCenterPos = new double[]{(cameraPos[0] + robotBRPos[0])/2, (cameraPos[1] + robotBRPos[1])/2};

            telemetry.addData("\nXAprilTagPos", aprilTagPos[0]);
            telemetry.addData("YAprilTagPos", aprilTagPos[1]);

            telemetry.addData("\nxBackDist", xBackDist);
            telemetry.addData("yBackDist", yBackDist);
            telemetry.addData("xCameraPos", cameraPos[0]);
            telemetry.addData("yCameraPos", cameraPos[1]);

            telemetry.addData("\nxRobotTRDist", xRobotTRDist);
            telemetry.addData("yRobotTRDist", yRobotTRDist);
            telemetry.addData("xRobotTRPos", robotTRPos[0]);
            telemetry.addData("yRobotTRPos", robotTRPos[1]);

            telemetry.addData("\nxRobotBRDist", xRobotBRDist);
            telemetry.addData("yRobotBRDist", yRobotBRDist);
            telemetry.addData("xRobotBRPos", robotBRPos[0]);
            telemetry.addData("yRobotBRPos", robotBRPos[1]);

            telemetry.addData("\nxRobotCenterPos", robotCenterPos[0]);
            telemetry.addData("yRobotCenterPos", robotCenterPos[1]);

            drive.setPoseEstimate(new Pose2d(robotCenterPos[0], robotCenterPos[1], -yaw));
        }
    }

    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection getDetections(){
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            desiredTag = detection;
            break;
        }
        return desiredTag;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
}
