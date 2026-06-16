package org.firstinspires.ftc.teamcode.Testers;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimelightRunner;

@TeleOp (group = "enigma", name = "LimelightTester")
public class LimelightTester extends LinearOpMode {
    public LimelightRunner runOLime;
    public void runOpMode(){
        runOLime = new LimelightRunner(hardwareMap.get(Limelight3A.class, "limelight"));
        waitForStart();
        while(opModeIsActive()){
            final double convert/*inches per meter*/ = 39.3700787402;
            Pose limePut;
            if ((limePut = runOLime.getBotPose()) != null) {
                telemetry.addData("Limelight position output x", limePut.getX() * convert);
                telemetry.addData("Limelight position output y", limePut.getY() * convert);
                telemetry.addData("Limelight heading output", limePut.getHeading() * convert);
            }
            telemetry.update();
        }
    }
}