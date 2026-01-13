package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class followPathAuto extends LinearOpMode {
    DcMotor intakeMotor;
    DcMotor outtakeMotor;
    DcMotor outtakeMotor2;
    Servo outtakeServo;
    public boolean isRed = false;
    public String color = "blue";
    public boolean colorSet = false;
    public double timer = 0;
    public double timer2;
    public boolean timerSet = true;
    public Pose[] positionsBlue = {new Pose(62,8, 111), new Pose(36.5,35.5, 180), new Pose(7,35.5,315), new Pose(62,8, 111), new Pose(36.5,60, 180), new Pose(14,60, 300), new Pose(62,8,111)};
    public Pose[] positionsRed = {new Pose()};
    private Follower robot;
    public Pose[] positions;
    public int posIndex = 0;
    enum PartsOfAuto {
        move,
        intake,
        shoot
    }
    PartsOfAuto partsOfAuto = PartsOfAuto.move;
    public int numberOfArtifacts = 3;
    public void intake (){
        intakeMotor.setPower(0.8);
    }
    public void outtake (){
        outtakeMotor.setPower(-0.7);
        outtakeMotor2.setPower(0.7);
        if (timer <= System.currentTimeMillis() - 300){
            outtakeServo.setPosition(0.88);
            timerSet = false;
            timer2 = System.currentTimeMillis();
            numberOfArtifacts--;
        }else if (!timerSet && timer2 <= System.currentTimeMillis()){
            timer = System.currentTimeMillis();
            outtakeServo.setPosition(0.98);
            timerSet = true;
        }
    }
    public void runOpMode (){
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        while (!opModeIsActive()){
            if (gamepad1.a){
                isRed = true;
                colorSet = true;
                color = "red";
                positions = positionsBlue;
            }
            if (gamepad1.b){
                isRed = false;
                colorSet = true;
                color = "blue";
                positions = positionsRed;
            }
            if (!colorSet){
                telemetry.addLine("Looking for color. Press a for red, press b for blue");
            }else {
                telemetry.addLine("Team color is " + color + " .");
            }
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            if (robot.getPose() == positions[posIndex]){
                posIndex++;
            }
            if (partsOfAuto == PartsOfAuto.shoot){
                outtake();
                if (numberOfArtifacts == 0){
                    partsOfAuto = PartsOfAuto.move;
                }
            }
            if (partsOfAuto == PartsOfAuto.intake){
                intake();
                numberOfArtifacts = 3;
            }
            if (partsOfAuto == PartsOfAuto.move || partsOfAuto == PartsOfAuto.intake) {
                robot.followPath(robot.pathBuilder().addPath(new BezierLine(robot.getPose(), positions[posIndex])).setLinearHeadingInterpolation(robot.getHeading(), positions[posIndex].minus(robot.getPose()).getAsVector().getTheta()).build());
                if (posIndex%3 == 0){
                    partsOfAuto = PartsOfAuto.shoot;
                }else if (posIndex%3 == 2){
                    partsOfAuto = PartsOfAuto.intake;
                }
            }
        }
    }
}