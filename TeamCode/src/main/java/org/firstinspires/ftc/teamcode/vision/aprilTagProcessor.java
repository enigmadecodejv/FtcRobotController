package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class aprilTagProcessor {
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    public HardwareMap hardwareMap;
    public boolean targetFound = false;
    public double sizeAt1INCH;

    public aprilTagProcessor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /*public class polarCords {
        double angle;
        double distance;

        public polarCords(double distance, double angle) {
            this.angle = angle;
            this.distance = distance;
        }
    }*/
    public void initialize(){
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
    }
    public PurpleOrGreen [] motif (){
        PurpleOrGreen [] motif = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections){
            if (detection.metadata != null){

            }
            if (detection.id == 21){
                motif = new PurpleOrGreen[]{PurpleOrGreen.green, PurpleOrGreen.purple, PurpleOrGreen.purple};
            }else if (detection.id == 22){
                motif = new PurpleOrGreen[]{PurpleOrGreen.purple, PurpleOrGreen.green, PurpleOrGreen.purple};
            }else if (detection.id == 23){
                motif = new PurpleOrGreen[]{PurpleOrGreen.purple, PurpleOrGreen.purple, PurpleOrGreen.green};
            }
        }
        return motif;
    }
    /*public polarCords aprilCords() {
        targetFound = false;
        polarCords coords = null;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //Check if this is the tag we want
                if (detection.id == 20) {
                    coords = new polarCords(detection.rawPose.x, Math.atan(detection.rawPose.y/detection.rawPose.x));
                    // Yes, we want to use this tag.
                    targetFound = true;
                    break;  // don't look any further.
                } else {
                }
            }
        }
        return coords;
    }*/
}