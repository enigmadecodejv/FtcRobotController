package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightRunner {
    Limelight3A limelight;
    public LimelightRunner(Limelight3A limelight){
        this.limelight = limelight;
        limelight.start();
        limelight.pipelineSwitch(0);//Red goal pipeline
    }
    public double getDX(int goalTag){
        double DX;
        LLResult result = limelight.getLatestResult();
        DX = Double.NaN;
        if (result != null && result.isValid()) {
            DX = Double.NaN;
            // Get all detected AprilTags
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (f.getFiducialId() == goalTag) {
                    DX = f.getTargetXDegrees() /*+ limeDist*/;
                    Pose3D pose = f.getRobotPoseTargetSpace();
                    /*DX = f.getTargetXDegrees() -
                            Math.atan((pose.getPosition().y +
                            aprilShotDist)/pose.getPosition().x) -
                            Math.atan(pose.getPosition().y/pose.getPosition().x);*/
                }
            }
        }
        return DX;
    }
    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
}
