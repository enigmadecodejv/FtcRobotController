package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="FollowPathAuto", group="Enigma")
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
    public Pose[] positionsBlueFar = {new Pose(62,8, 111), new Pose(36.5,35.5, Math.toRadians(180)), new Pose(7,35.5,Math.toRadians(315)), new Pose(62,8, Math.toRadians(111)), new Pose(36.5,60, Math.toRadians(180)), new Pose(14,60, Math.toRadians(300)), new Pose(62,8,Math.toRadians(111))};
    public Pose[] positionsRedFar = {new Pose(80, 16, Math.toRadians(66)), new Pose(105, 35, 0), new Pose(96, 34, 0), new Pose(131, 35, 0), new Pose(80, 16, Math.toRadians(66)), new Pose(96, 59, 0), new Pose(96, 60, 0), new Pose(80, 16, Math.toRadians(66))};
    public Pose[] positionsBlueNear = {};
    public Pose[] positionsRedNear = {new Pose(94, 94, Math.toRadians(48))};
    private Follower robot;
    public Pose[] positions;
    public int posIndex = 0;
    public boolean ResetShoot = false;
    private Path forwards;
    public Pose robotPos;
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
        if (!ResetShoot){
            outtakeServo.setPosition(0.88);
            if (outtakeServo.getPosition() == 0.88) {
                numberOfArtifacts--;
                ResetShoot = true;
                timer = System.currentTimeMillis() + 1000;
            }
        } else {
            outtakeServo.setPosition(0.98);
            if (timer >= System.currentTimeMillis()) {
                if (outtakeServo.getPosition() == 0.98) {
                    intake();
                }
            } else {
                ResetShoot = false;
            }
        }
    }
    public void redOrBlue () {
        if (gamepad1.a){
            isRed = true;
            colorSet = true;
            color = "red";
            //positions = positionsBlue;
        }
        if (gamepad1.b){
            isRed = false;
            colorSet = true;
            color = "blue";
            //positions = positionsRed;
        }
        if (!colorSet){
            telemetry.addLine("Looking for color. Press a for red, press b for blue");
        }else {
            telemetry.addLine("Team color is " + color + " .");
        }
    }
    public void startPos () {
        if (colorSet) {
            telemetry.addLine("looking for starting pos press a for nearside, b for farside");
            if (color.equals("blue")) {
                if (gamepad1.b) {
                    robotPos = new Pose(56, 8, 90);
                }  else if (gamepad1.a) {
                    robotPos = new Pose(39, 136, 270);
                }
            } else if (color.equals("red")) {
                if (gamepad1.b) {
                    robotPos = new Pose(88, 8, 90);
                }  else if (gamepad1.a) {
                    robotPos = new Pose(104, 136, 270);
                }
            }
        }
    }
    public void runOpMode (){
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        while (!opModeIsActive()) {
            redOrBlue();
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            outtakeMotor.setPower(-0.7);
            outtakeMotor2.setPower(0.7);
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
                numberOfArtifacts = 2;
            }
            if (partsOfAuto == PartsOfAuto.move || partsOfAuto == PartsOfAuto.intake) {
                if (posIndex > 0) {
                    robotPos = positions[posIndex - 1];
                }
                forwards = new Path(new BezierLine(robotPos, positions[posIndex]));
                forwards.setConstantHeadingInterpolation(0);
                if (posIndex%3 == 0){
                    partsOfAuto = PartsOfAuto.shoot;
                }else if (posIndex%3 == 2){
                    partsOfAuto = PartsOfAuto.intake;
                }
            }
        }
        robot.update();
        drawOnlyCurrent();
    }
}