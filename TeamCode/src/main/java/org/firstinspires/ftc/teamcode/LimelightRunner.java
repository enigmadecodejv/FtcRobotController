package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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
                }
            }
        }
        return DX;
    }
    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
}
