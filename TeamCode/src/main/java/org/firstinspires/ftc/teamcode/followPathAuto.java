package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
@Configurable
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
    public Pose[] positionsBlueFar = {
            new Pose(40,35, Math.toRadians(180)),
            new Pose(10,35,Math.toRadians(180)),
            new Pose(55, 58, Math.toRadians(180)),
            new Pose(10,60, Math.toRadians(180))
    };
    public Pose[] positionsRedFar = {
            new Pose(97, 35, 0),
            new Pose(125, 36, 0),
            new Pose(83, 55, 0),
            new Pose(125, 60, Math.toRadians(0))
    };
    public Pose[] positionsBlueNear = {
            new Pose(48, 84, Math.toRadians(180)),
            new Pose(15, 84, Math.toRadians(180)),
            new Pose(58, 63, Math.toRadians(180)),
            new Pose(14, 60, Math.toRadians(180))};
    public Pose[] positionsRedNear = {
            new Pose(106, 84, 0),
            new Pose(125, 84, 0),
            new Pose(125, 59, 0),
            new Pose(125, 60, 0)
    };
    private Follower robot;
    public Pose[] positions;
    public int posIndex = 0;
    public boolean ResetShoot = false;
    private PathChain forwards;
    public Path scorePreload = null;
    public Pose robotPos = null;
    private TelemetryManager manager;
    public double integral = 0;
    public double derivative;
    public double motorSpeed;
    public double error;
    public double timerPI;
    public double lastError = 33778;
    public static double kp = 0.01;
    public static double ti = 60;
    public static double td = 0;
    public double integral2 = 0;
    public static double farSpeed = 1410;
    public static double closeSpeed = 1150;
    public double speedPID = closeSpeed;
    public double targetPower;
    enum PartsOfAuto {
        move,
        intake,
        shoot
    }
    PartsOfAuto partsOfAuto = PartsOfAuto.shoot;
    public Pose lastPose;
    public int numberOfArtifacts = 2;
    public double outtakePID(double targetSpeed){
        error = targetSpeed - outtakeLeft.getVelocity();
        integral += error;
        integral2 += error;
        timerPI += 1;
        if (lastError != 33778) {
            derivative = (error - lastError) / timerPI;
        }
        lastError = error;
        if (ti == 0){
            targetPower = kp*(error + td*derivative);
        }else{
            targetPower = kp*(error + integral/ti + td*derivative);
        }
        if(targetPower > 1){
            targetPower = 1;
        }else if (targetPower < -1){
            targetPower = -1;
        }
        return targetSpeed;
    }
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
            outtakeRight.setPower(outtakeLeft.getPower());
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
                shootPos = new Pose(85, 16, Math.toRadians(240));
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
            outtakeRight.setPower(outtakeLeft.getPower());
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

        intakeLeft = hardwareMap.get(DcMotor.class, "IntakeLeft");
        robot = Constants.createFollower(hardwareMap);
        intakeRight = hardwareMap.get(DcMotor.class, "IntakeRight");

        outtakeRight = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");

        outtakeGate = hardwareMap.get(Servo.class, "OuttakeGate");
        manager = PanelsTelemetry.INSTANCE.getTelemetry();

        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (isFar) {
            speedPID = farSpeed;
        } else {
            speedPID = closeSpeed;
        }
        outtakeLeft.setVelocity(speedPID);
        waitForStart();
        Pose l;
        while (opModeIsActive()) {
            PIDCoefficients pidCoefficients = outtakeLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidCoefficients.p = kp;
            pidCoefficients.i = ti;
            pidCoefficients.d = td;
            outtakeLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            outtakeRight.setPower(outtakeLeft.getPower());
            manager.addData("MotorVelocity", outtakeLeft.getVelocity());
            manager.addData("Integral", integral);
            manager.addData("Integral2", integral2);
            manager.addData("error", error);
            if (PIOuttake.ti == 0){
                manager.addData("Integral/ti", 0);
            }else {
                manager.addData("Integral/ti", integral / ti);
            }
            outtakeGate.setPosition(0.7);
            if (partsOfAuto == PartsOfAuto.move && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                goToNextPose(getNextPose());
                partsOfAuto = PartsOfAuto.intake;

            } else if (partsOfAuto == PartsOfAuto.intake && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                intake();
                goToNextPose(getNextPose());
                numberOfArtifacts = 3;
                partsOfAuto = PartsOfAuto.shoot;

            } else if (partsOfAuto == PartsOfAuto.shoot && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                turnOffIntake();
                goToNextPose(shootPos);
                outtake();
                partsOfAuto = PartsOfAuto.move;
            }
            manager.update();
        }
    }
}