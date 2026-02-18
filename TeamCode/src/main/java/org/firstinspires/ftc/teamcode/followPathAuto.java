package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="FollowPathAuto", group="Enigma")
public class followPathAuto extends LinearOpMode {
    DcMotor intakeLeft;
    DcMotor intakeRight;
    public int tickNum = 0;
    public boolean isFar;
    public Timer timer;
    DcMotor outtakeRight;
    DcMotorEx outtakeLeft;
    Servo outtakeGate;
    public boolean isRed = false;
    public String color = "blue";
    public boolean colorSet = false;
    public boolean isStartSet = false;
    public double timer2;
    public boolean timerSet = true;
    public Pose[] positionsBlueFar = {new Pose(59,16, Math.toRadians(113)), new Pose(36.5,35.5, Math.toRadians(180)), new Pose(7,35.5,Math.toRadians(315)), new Pose(62,8, Math.toRadians(111)), new Pose(36.5,60, Math.toRadians(180)), new Pose(14,60, Math.toRadians(300)), new Pose(62,8,Math.toRadians(111))};
    public Pose[] positionsRedFar = {new Pose(80, 16, Math.toRadians(66)), new Pose(105, 35, 0), new Pose(131, 35, 0), new Pose(80, 16, Math.toRadians(66)), new Pose(130, 60, 0), new Pose(80, 16, Math.toRadians(66))};
    public Pose[] positionsBlueNear = {new Pose(59, 85, Math.toRadians(138))};
    public Pose[] positionsRedNear = {new Pose(85, 85, Math.toRadians(48))};
    private Follower robot;
    public Pose[] positions;
    public int posIndex = 0;
    public boolean ResetShoot = false;
    private PathChain forwards;
    public Path scorePreload = null;
    public Pose robotPos = null;
    enum PartsOfAuto {
        move,
        intake,
        shoot
    }
    PartsOfAuto partsOfAuto = PartsOfAuto.shoot;
    public Pose lastPose;
    public int numberOfArtifacts = 2;
    public void intake (){
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }
    public void turnOffIntake () {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
    public void outtake () {
        timer.resetTimer();
        outtakeGate.setPosition(0.35);
        while (numberOfArtifacts > 0 && opModeIsActive()) {
            intake();
            if (timer.getElapsedTimeSeconds() < 1) {
                turnOffIntake();
                numberOfArtifacts = 0;
            }
        }
    }
    public void redOrBlue () {
        if (colorSet) {
            return;
        }
        if (gamepad1.a) {
            isRed = true;
            colorSet = true;
            color = "red";
            //positions = positionsBlue;
        }
        if (gamepad1.b) {
            isRed = false;
            colorSet = true;
            color = "blue";
            //positions = positionsRed;
        }
        if (!colorSet) {
            telemetry.addLine("Looking for color. Press a for red, press b for blue");
        } else {
            telemetry.addLine("Team color is " + color + " .");
            waitForButton();
        }
    }
    public void setStartPos () {
        if (!colorSet) {
            return;
        } else if (isStartSet) {
            return;
        }
        telemetry.addLine("looking for starting pos press a for nearside, b for farside");
        if (color.equals("blue")) {
            if (gamepad1.b) {
                robotPos = new Pose(56, 8, Math.toRadians(90));
                isFar = true;
            } else if (gamepad1.a) {
                robotPos = new Pose(39, 136, Math.toRadians(270));
                isFar = false;
            }
        } else if (color.equals("red")) {
            if (gamepad1.b) {
                robotPos = new Pose(85, 8.75, Math.toRadians(90));
                isFar = true;
            } else if (gamepad1.a) {
                robotPos = new Pose(104, 136, Math.toRadians(270));
                isFar = false;
            }
        }
        if (robotPos != null) {
            robot.setStartingPose(new Pose(72, 72, 90));
            lastPose = new Pose(72, 72, 90);
            robot.update();
            isStartSet = true;
        }
    }
    public void waitForButton () {
        while (gamepad1.a || gamepad1.b) {
            sleep(10);
        }
    }
    public void goToPos (Pose endPose) {
        forwards = robot.pathBuilder()
                .addPath(new BezierLine(lastPose, endPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), endPose.getHeading())
                .build();
        robot.followPath(forwards, true);
        robot.update();
        while (robot.isBusy() && opModeIsActive()) {
            robot.update();
        }
        lastPose = endPose;
    }
    public Pose getNextPose () {
        Pose p;
        if (color.equals("red")) {
            if (isFar) {
                p = positionsRedFar [posIndex];
            }  else  {
                p = positionsRedNear [posIndex];
            }
        } else  {
            if (isFar) {
                p = positionsBlueFar[posIndex];
            } else {
                p = positionsBlueNear[posIndex];

            }
        }
        posIndex++;
        return p;
    }
    public Pose goToNextPose () {
        telemetry.addLine("here");
        telemetry.update();
        Pose p = getNextPose();
        telemetry.addData("p", p);
        telemetry.update();
        goToPos(p);
        telemetry.addLine("we somehow got here");
        telemetry.update();
        return p;
    }

    //set start pos
    //start by going to shoot pos
    //shoot 2 balls
    //pickup balls
    //go to shoot pos
    //pickup balls
    public void runOpMode (){
        robot = Constants.createFollower(hardwareMap);
        intakeLeft = hardwareMap.get(DcMotor.class, "IntakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "IntakeRight");
        outtakeRight = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeGate = hardwareMap.get(Servo.class, "OuttakeGate");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer = new Timer();
        timer.resetTimer();

        while (!opModeIsActive()) {
            redOrBlue();
            setStartPos();
            if (colorSet && isStartSet) {
                telemetry.addData("color", color);
                telemetry.addData("isFar is", isFar);
            }
            telemetry.update();
        }
        waitForStart();
        Pose l;
        while (opModeIsActive()) {
            if (isFar) {
                outtakeLeft.setVelocity(1000);
                outtakeRight.setPower(outtakeLeft.getPower());
            } else {
                outtakeLeft.setVelocity(1000);
                outtakeRight.setPower(outtakeLeft.getPower());
            }
            outtakeGate.setPosition(0.7);
            if (!robot.isBusy() && partsOfAuto == PartsOfAuto.move) {
                goToNextPose();
                partsOfAuto = PartsOfAuto.intake;
            } else if (!robot.isBusy() && partsOfAuto == PartsOfAuto.intake) {
                intake();
                goToNextPose();
                turnOffIntake();
                numberOfArtifacts = 3;
                partsOfAuto = PartsOfAuto.shoot;
            } else if (!robot.isBusy() && partsOfAuto == PartsOfAuto.shoot) {
                goToNextPose();
                outtake();
                partsOfAuto = PartsOfAuto.move;
            }
        }
    }
}