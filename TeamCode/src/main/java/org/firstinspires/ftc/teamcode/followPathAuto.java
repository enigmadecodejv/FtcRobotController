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

import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

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
    PIOuttake outtakeCode;
    public boolean isRed = false;
    public String color = "blue";
    public boolean colorSet = false;
    public boolean isStartSet = false;
    public double timer2;
    public boolean timerSet = true;
    public Pose shootPos;
    public Pose[] positionsBlueFar = {new Pose(40,35, Math.toRadians(180)), new Pose(10,35,Math.toRadians(180)), new Pose(55,58, Math.toRadians(180)), new Pose(10,60, Math.toRadians(180))};
    public Pose[] positionsRedFar = {new Pose(97, 35, 0), new Pose(125, 36, 0), new Pose(83, 55, 0), new Pose(125, 60, Math.toRadians(0))};
    public Pose[] positionsBlueNear = {new Pose(48, 84, Math.toRadians(180)), new Pose(15, 84, Math.toRadians(180)), new Pose(58, 63, Math.toRadians(180)), new Pose(14, 60, Math.toRadians(180))};
    public Pose[] positionsRedNear = {new Pose(106, 84, 0), new Pose(125, 84, 0), new Pose(125, 59, 0), new Pose(125, 60, 0)};
    private Follower robot;
    public Pose[] positions;
    public int posIndex = 0;
    public boolean ResetShoot = false;
    private PathChain forwards;
    public Path scorePreload = null;
    public Pose robotPos = null;
    private TelemetryManager manager;
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
            if (timer.getElapsedTimeSeconds() >= 1) {
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
                robotPos = new Pose(56, 8, Math.toRadians(270));
                shootPos = new Pose(59,16, Math.toRadians(293));
                isFar = true;
            } else if (gamepad1.a) {
                robotPos = new Pose(39, 136, Math.toRadians(90));
                shootPos = new Pose(59, 85, Math.toRadians(138));
                isFar = false;
            }
        } else if (color.equals("red")) {
            if (gamepad1.b) {
                robotPos = new Pose(85, 8.75, Math.toRadians(270));
                shootPos = new Pose(85, 16, Math.toRadians(246));
                isFar = true;
            } else if (gamepad1.a) {
                robotPos = new Pose(104, 136, Math.toRadians(90));
                shootPos = new Pose(85, 85, Math.toRadians(48));
                isFar = false;
            }
        }
        if (robotPos != null) {
            robot.setStartingPose(robotPos);
            lastPose = robotPos;
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
        robot.followPath(forwards, 0.5, true);
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
    public Pose goToNextPose (Pose nextPos) {
        telemetry.addLine("here");
        telemetry.update();
        telemetry.addData("p", nextPos);
        telemetry.update();
        goToPos(nextPos);
        telemetry.addLine("we somehow got here");
        telemetry.update();
        return nextPos;
    }

    //set start pos
    //start by going to shoot pos
    //shoot 2 balls
    //pickup balls
    //go to shoot pos
    //pickup balls
    public void runOpMode (){
        outtakeCode = new PIOuttake(hardwareMap, gamepad1);
        manager = PanelsTelemetry.INSTANCE.getTelemetry();
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
            manager.addData("MotorVelocity", outtakeLeft.getVelocity());
            manager.addData("Integral", outtakeCode.integral);
            manager.addData("Integral2", outtakeCode.integral2);
            manager.addData("error", outtakeCode.error);
            if (isFar) {
                outtakeCode.speedPID = PIOuttake.farSpeed;
                double motorPower = outtakeCode.outtakePID(outtakeCode.speedPID);
                outtakeLeft.setPower(motorPower);
                outtakeRight.setPower(motorPower);
            } else {
                outtakeCode.speedPID = PIOuttake.closeSpeed;
                double motorPower = outtakeCode.outtakePID(outtakeCode.speedPID);
                outtakeLeft.setPower(motorPower);
                outtakeRight.setPower(motorPower);
            }
            outtakeGate.setPosition(0.7);
            if (partsOfAuto == PartsOfAuto.move && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - outtakeCode.speedPID) < 20) {
                goToNextPose(getNextPose());
                partsOfAuto = PartsOfAuto.intake;

            } else if (partsOfAuto == PartsOfAuto.intake && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - outtakeCode.speedPID) < 20) {
                intake();
                goToNextPose(getNextPose());
                numberOfArtifacts = 3;
                partsOfAuto = PartsOfAuto.shoot;

            } else if (partsOfAuto == PartsOfAuto.shoot && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - outtakeCode.speedPID) < 20) {
                turnOffIntake();
                goToNextPose(shootPos);
                outtake();
                partsOfAuto = PartsOfAuto.move;
            }
            manager.update();
        }
    }
}