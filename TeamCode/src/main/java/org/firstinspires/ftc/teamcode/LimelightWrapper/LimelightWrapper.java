package org.firstinspires.ftc.teamcode.LimelightWrapper;

import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import javax.annotation.CheckForNull;

public class AprilTagInfo {
    boolean isValid = false;
    int tagId = -1;
    double azimuthOffsetDeg = Double.NaN;
    Pose3D Pose = new Pose3D(
            new Position(DistanceUnit.MM, Double.NaN, Double.NaN, Double.NaN, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, Double.NaN, Double.NaN, Double.NaN, 0)
    );
    Pose3D


    public AprilTagInfo() {}
    public AprilTagInfo(LLResultTypes.FiducialResult result) {
        isValid = true;

    }

    private set
}

public class LimelightWrapper {
    Limelight3A m_limelight;
    int m_pipelineIndex = 0;

    // Construct LimelightWrapper
    public LimelightWrapper(Limelight3A limelight){
        this.m_limelight = limelight;
        m_limelight.start();
        m_limelight.pipelineSwitch(m_pipelineIndex);
    }

    public void switchPipeline(int newIndex){
        m_pipelineIndex = newIndex;
        m_limelight.pipelineSwitch(m_pipelineIndex);
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
