package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LimelightRunner;
import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
@Configurable
@Autonomous(name="FollowPathAuto", group="Enigma")
public class followPathAuto extends LinearOpMode {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    DcMotor intakeLeft;
    DcMotor intakeRight;
    Limelight3A limelight;
    public boolean isFar;
    public Timer timer;
    DcMotor outtakeRight;
    DcMotorEx outtakeLeft;
    Servo outtakeGate;
    LimelightRunner runOLime;
    public boolean isRed = false;
    public String color = "blue";
    public boolean colorSet = false;
    public boolean isStartSet = false;
    public Pose shootPos;
    public Pose[] positionsBlueFar = {
            new Pose(40,35, Math.toRadians(180)),
            new Pose(10,35,Math.toRadians(180)),
            new Pose(55, 58, Math.toRadians(180)),
            new Pose(10,60, Math.toRadians(180)),
            new Pose(36, 84, Math.toRadians(180)),
            new Pose(13, 84, Math.toRadians(180)),
            new Pose(39,12, Math.toRadians(180))
    };
    public Pose[] positionsRedFar = {
            new Pose(97, 35, 0),
            new Pose(125, 36, 0),
            new Pose(83, 55, 0),
            new Pose(125, 60, Math.toRadians(0)),
            new Pose(97, 84, Math.toRadians(0)),
            new Pose(130, 84, Math.toRadians(0)),
            new Pose(114, 12, Math.toRadians(0))
    };
    public Pose[] positionsBlueNear = {
            new Pose(48, 84, Math.toRadians(180)),
            new Pose(15, 84, Math.toRadians(180)),
            new Pose(58, 63, Math.toRadians(180)),
            new Pose(14, 60, Math.toRadians(180)),
            new Pose(40,35, Math.toRadians(180)),
            new Pose(10,35, Math.toRadians(180)),
            new Pose(24, 84, Math.toRadians(180))
    };
    public Pose[] positionsRedNear = {
            new Pose(106, 84, 0),
            new Pose(125, 84, 0),
            new Pose(125, 59, 0),
            new Pose(125, 60, 0),
            new Pose(97, 35, 0),
            new Pose(125, 36, 0),
            new Pose(120, 90, 0)
    };
    private Follower robot;
    public int posIndex = 0;
    private PathChain forwards;
    public Pose robotPos = null;
    private TelemetryManager manager;
    public double integral = 0;
    public double derivative;
    public double error;
    public double timerPI;
    public double lastError = 33778;
    public static double kp = 0.01;
    public static double ti = 60;
    public static double td = 0;
    public static double kp_DX = 0.05;
    public double integral2 = 0;
    public static double farSpeed = 1410;
    public static double closeSpeed = 1150;
    public double speedPID = closeSpeed;
    public double targetPower;
    public int goalTag;
    public double DX;
    public int loopNum = 0;
    enum PartsOfAuto {
        move,
        intake,
        shoot
    }
    PartsOfAuto partsOfAuto = PartsOfAuto.shoot;
    public Pose lastPose;
    public int numberOfArtifacts = 3;
    public double outtakePID(double targetSpeed){
        error = targetSpeed - outtakeLeft.getVelocity();
        integral += error;
        integral2 += error;
        timerPI += 1;
        if (lastError != 33778) {
            derivative = (error - lastError) /*/ timerPI*/;
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
        return targetPower;
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
        outtakeGate.setPosition(0.0);
        timer.resetTimer();
        while (numberOfArtifacts > 0 && opModeIsActive()) {
            double outtakePower = outtakePID(speedPID);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);
            robot.update();
            DX = runOLime.getDX(goalTag);
            if (DX != DX){
                telemetry.addLine("DX is NaN");
            }
            if (Math.abs(DX) > 1.0) {
                double turn = DX * kp_DX;
                if (Math.abs(turn) > 0.75){
                    turn = 0.75 * turn/Math.abs(turn);
                }
                leftFrontDrive.setPower(+turn);
                leftBackDrive.setPower(+turn);
                rightFrontDrive.setPower(-turn);
                rightBackDrive.setPower(-turn);
                turnOffIntake();
                timer.resetTimer();
            } else if (!robot.isBusy()) {
                intake();
                outtakeRight.setPower(outtakeLeft.getPower());
                if (timer.getElapsedTimeSeconds() >= 1) {
                    telemetry.addLine("yay");
                    turnOffIntake();
                    numberOfArtifacts = 0;
                }
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
            goalTag = 24;
            runOLime.switchPipeline(0);
            //positions = positionsBlue;
        }
        if (gamepad1.b) {
            isRed = false;
            colorSet = true;
            color = "blue";
            goalTag = 20;
            runOLime.switchPipeline(1);
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
    public void goToPos (Pose endPose, boolean holdEnd) {
        forwards = robot.pathBuilder()
                .addPath(new BezierLine(lastPose, endPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), endPose.getHeading())
                .build();
        robot.followPath(forwards, 0.5, holdEnd);
        robot.update();
        while (robot.isBusy() && opModeIsActive()) {
            double outtakePower = outtakePID(speedPID);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);
            outtakeRight.setPower(outtakeLeft.getPower());
            robot.update();
        }
        lastPose = endPose;
    }
    public Pose getNextPose () {
        Pose p;
        if (color.equals("red")) {
            if (isFar) {
                p = positionsRedFar[posIndex];
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
    public Pose goToNextPose (Pose nextPos, boolean holdEnd) {
        goToPos(nextPos, holdEnd);
        return nextPos;
    }

    //set start pos
    //start by going to shoot pos
    //shoot 2 balls
    //pickup balls
    //go to shoot pos
    //pickup balls
    public void runOpMode (){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        runOLime = new LimelightRunner(limelight);
        intakeLeft = hardwareMap.get(DcMotor.class, "IntakeLeft");
        robot = Constants.createFollower(hardwareMap);
        intakeRight = hardwareMap.get(DcMotor.class, "IntakeRight");

        outtakeRight = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");

        outtakeGate = hardwareMap.get(Servo.class, "OuttakeGate");
        manager = PanelsTelemetry.INSTANCE.getTelemetry();

        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        waitForStart();
        while (opModeIsActive()) {
            /*PIDCoefficients pidCoefficients = outtakeLeft.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            pidCoefficients.p = kp;
            pidCoefficients.i = ti;
            pidCoefficients.d = td;
            outtakeLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
            outtakeRight.setPower(outtakeLeft.getPower());*/
            double outtakePower = outtakePID(speedPID);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);
            manager.addData("MotorVelocity", outtakeLeft.getVelocity());
            manager.addData("Integral", integral);
            manager.addData("Integral2", integral2);
            manager.addData("error", error);
            manager.addData("outtakePower", outtakePower);
            manager.addData("Pedro x", robot.getPose().getX());
            manager.addData("Pedro y", robot.getPose().getY());
            manager.addData("Pedro heading", robot.getPose().getHeading());
            if (PIOuttake.ti == 0){
                manager.addData("Integral/ti", 0);
            }else {
                manager.addData("Integral/ti", integral / ti);
            }
            outtakeGate.setPosition(0.2);
            if (partsOfAuto == PartsOfAuto.move && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                goToNextPose(getNextPose(), true);
                partsOfAuto = PartsOfAuto.intake;

            } else if (partsOfAuto == PartsOfAuto.intake && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                intake();
                goToNextPose(getNextPose(), true);
                numberOfArtifacts = 3;
                partsOfAuto = PartsOfAuto.shoot;

            } else if (partsOfAuto == PartsOfAuto.shoot && !robot.isBusy() && Math.abs(outtakeLeft.getVelocity() - speedPID) < 20) {
                turnOffIntake();
                goToNextPose(shootPos, false);
                outtake();
                partsOfAuto = PartsOfAuto.move;
            }
            manager.update();
            if (loopNum < 0) {
                telemetry.addData("loopNum", loopNum);
            }
        }
    }
}