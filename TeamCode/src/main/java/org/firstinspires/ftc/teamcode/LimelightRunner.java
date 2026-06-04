package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
                }
            }
        }
        return DX;
    }
    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public Pose getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D BotPose = result.getBotpose();
            return new Pose(BotPose.getPosition().x, BotPose.getPosition().y, BotPose.getOrientation().getYaw(AngleUnit.RADIANS));
        }
        return null;
    }
}
