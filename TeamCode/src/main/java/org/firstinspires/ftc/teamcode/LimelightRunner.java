package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import javax.annotation.CheckForNull;

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

    @CheckForNull
    public Pose getBotPose/*may return null*/() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() /*&& result.getFiducialResults().size() >= 2*/) {
            Pose3D BotPose = result.getBotpose();
            return new Pose(BotPose.getPosition().y + 1.8/*limelight outputs between ~ -1.8 and 1.8, so adjust*//*Limelight also outputs what pedro thinks is x as y and y as x*/,
                    BotPose.getPosition().x * (-1) + 1.8/*limelight outputs a value between 1.8 and ~ -1.8, so adjust to meters*/,
                    BotPose.getOrientation().getYaw(AngleUnit.DEGREES));
        }
        //Return null if the result is not valid
        return null;
    }
}
