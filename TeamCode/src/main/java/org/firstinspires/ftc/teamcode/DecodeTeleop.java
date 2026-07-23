package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drive.NaNTurnBehavior;
import org.firstinspires.ftc.teamcode.Drive.decodeDriveCode;
import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.intake.decodeIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.io.FileNotFoundException;
import java.util.ArrayList;

@Configurable
@TeleOp(name="DecodeTeleop", group="Enigma")
public class DecodeTeleop extends LinearOpMode {
    //public GoBildaPinpointDriver pinpoint;
    public PathChain path;
    public FileReadWriter fileReaderWriter;
    public Follower robot;
    decodeDriveCode driveCode;
    decodeIntake intakeCode;
    PIOuttake outtakeCode;
    DcMotorEx outtakeMotor;
    DcMotor outtakeMotor2;
    Gamepad pastGamepad1;
    private TelemetryManager manager;
    Limelight3A limelight;
    LimelightRunner runOLime;
    public double DX = 0;
    public double DIST = 0;
    public double outtakeMotorVelocity = 0;
    public static double kP_TURN = 0.03;
    public static double tI_TURN = 960;
    public double integralTurn = 0;
    public double MAXTURN = 0.75;

    public boolean isPipelineSet = false;
    public String[] pipelineNames = {"CRIWholeField", "CRIRedAlliance", "CRIBlueAlliance", "CRIObeliskTest"};
    public int pipelineIndex = 0;

    public String[] tagNames = {"Goal", "Prism"};
    public int[] redTagIDs = {24, 14};
    public int[] blueTagIDs = {20, 10};

    public boolean isRed = true; // false is blue team
    public boolean isGoal = true; // false if pointing at prism
    public boolean isColorSet = false;

    private void setTurnInPlace(double turn) {
        if (Math.abs(turn) > MAXTURN){
            turn = MAXTURN * turn/Math.abs(turn);
        }
        driveCode.leftFrontDrive.setPower(+turn);
        driveCode.leftBackDrive.setPower(+turn);
        driveCode.rightFrontDrive.setPower(-turn);
        driveCode.rightBackDrive.setPower(-turn);
    }
    private void localizePinpoint(Pose limePose/*in meters and radians*/){
        if (limePose != null) {
            final double convert/*inches per meter*/ = 39.3700787402;
            robot.setPose(new Pose(limePose.getX() * convert, limePose.getY() * convert, limePose.getHeading()));
        }
    }
    private void setPipeline() {
        if (isPipelineSet || !isColorSet) { return; }

        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addLine("Press dpad up/down to cycle pipelines and right trigger to select");
        telemetry.addData("Number pipelines", pipelineNames.length);
        telemetry.addData("Current pipeline index", pipelineIndex);
        telemetry.addData("Current pipeline name", pipelineNames[pipelineIndex]);

        if (gamepad1.dpad_up && !pastGamepad1.dpad_up && (pipelineIndex < (pipelineNames.length-1))) {
            pipelineIndex++;
        } else if (gamepad1.dpad_down && !pastGamepad1.dpad_down && (pipelineIndex > 0)){
            pipelineIndex--;
        }

        if (gamepad1.right_trigger > 0.25 && pastGamepad1.right_trigger < 0.25) {
            isPipelineSet = true;
        }
    }
    public void setColor() {
        if (!isPipelineSet || isColorSet) { return; }

        telemetry.addLine("Press dpad up/down to cycle red/blue and right trigger to select");
        telemetry.addData("Current color", (isRed) ? "RED" : "BLUE");

        if ((gamepad1.dpad_up && !pastGamepad1.dpad_up) || (gamepad1.dpad_down && !pastGamepad1.dpad_down)) {
            isRed = !isRed;
        }

        if (gamepad1.right_trigger > 0.25 && pastGamepad1.right_trigger < 0.25) {
            isColorSet = true;
        }
    }
    private void checkToggleIsGoal() {
        if (gamepad1.x && !pastGamepad1.x) {
            isGoal = !isGoal;
        }
    }
    private int getTargetTagID() {
        if (isRed) {
            return (isGoal) ? redTagIDs[0] : redTagIDs[1];
        }
        return (isGoal) ? blueTagIDs[0] : blueTagIDs[1];
    }
    private String getTargetName() {
        if (isRed) {
            return (isGoal) ? "RED GOAL" : "RED PRISM";
        }
        return (isGoal) ? "BLUE GOAL" : "BLUE PRISM";
    }
    private void mainLoop() {
        /*if (gamepad2.left_trigger > 0.25){
            Pose limelightResult = runOLime.getBotPose();
            if (limelightResult != null){
                localizePinpoint(limelightResult);
            }
        }*/
//        if (gamepad1.x) {
//            currentPipeline++;
//            if (currentPipeline >= pipelineNames.length){
//                currentPipeline = pipelineNames.length - 1;
//            }else if (currentPipeline < 0){
//                currentPipeline = 0;
//            }
//            runOLime.switchPipeline(currentPipeline);
//        } else if (gamepad1.b){
//            currentPipeline--;
//            if (currentPipeline < 0){
//                currentPipeline = 0;
//            }else if (currentPipeline >= pipelineNames.length){
//                currentPipeline = pipelineNames.length - 1;
//            }
//            runOLime.switchPipeline(currentPipeline);
//        }

        checkToggleIsGoal();
        telemetry.addData("Target Name", getTargetName());
        int targetTagID = getTargetTagID();
        DX = runOLime.getDX(targetTagID);
        if (gamepad1.left_bumper) {
            /*if (!Double.isNaN(DX) && Math.abs(DX) <= 1.0) {
                setTurnInPlace(0);
            } else {*/
                double turn;
                if (!Double.isNaN(DX)) {
                    integralTurn += DX;
                }
                if (tI_TURN == 0) {
                    turn = kP_TURN * DX;
                }else{
                    turn = kP_TURN * DX + integralTurn/tI_TURN;
                }
                driveCode.runGivenTurn(turn, NaNTurnBehavior.SET_TO_JOYSTICK);
            //}
        } else {
            driveCode.runWheels();
        }
        intakeCode.intake();
        outtakeCode.runUsingPID();

        telemetry.addData("Pipeline for limelight is", pipelineNames[pipelineIndex]);
        /*telemetry.addData("motor velocity: ", outtakeCode.outtakeMotor.getVelocity());
        manager.addData("MotorVelocity", outtakeMotor.getVelocity());
        manager.addData("Integral", outtakeCode.integral);
        manager.addData("Integral2", outtakeCode.integral2);
        manager.addData("error", outtakeCode.error);
        manager.addData("derivative", outtakeCode.derivative);
        manager.addData("derivative*tD", outtakeCode.derivative * PIOuttake.td);
        manager.addData("integralTurn", integralTurn);
        if (tI_TURN != 0) {
            manager.addData("integralTurn/tI_TURN", integralTurn / tI_TURN);
        }else{
            manager.addData("integralTurn/tI_TURN", 0);
        }*/
        telemetry.addData("DX", DX);
        manager.addData("DX", DX);
        //telemetry.addData("DIST", DIST);
        Pose limelightOutput = runOLime.getBotPose();
        manager.addData("limelight Botpose x", Double.NaN);
        manager.addData("limelight Botpose y", Double.NaN);
        manager.addData("limelight Botpose heading", Double.NaN);
        if (limelightOutput != null) {
            telemetry.addData("limelight Botpose x", limelightOutput.getX());
            telemetry.addData("limelight Botpose y: ", limelightOutput.getY());
            telemetry.addData("limelight BotPose heading: ", limelightOutput.getHeading());
            manager.addData("limelight Botpose x", limelightOutput.getX());
            manager.addData("limelight Botpose y", limelightOutput.getY());
            manager.addData("limelight Botpose heading", limelightOutput.getHeading());
        }
        manager.addData("Pedropathing position x", robot.getPose().getX());
        manager.addData("Pedropathing position y", robot.getPose().getY());
        manager.addData("Pedropathing heading", robot.getHeading());
        /*telemetry.addData("OuttakeMotor Current", outtakeCode.outtakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("OuttakeMotor Power", outtakeCode.outtakeMotor.getPower());
        telemetry.addData("OuttakeMotor2 Power", outtakeCode.outtakeMotor.getPower());
        telemetry.addData("FrontLeft drive power", driveCode.leftFrontDrive.getPower());
        telemetry.addData("leftBack drive power", driveCode.leftBackDrive.getPower());
        telemetry.addData("rightFront drive power", driveCode.rightFrontDrive.getPower());
        telemetry.addData("rightBack drive power", driveCode.rightBackDrive.getPower());
        manager.addData("OuttakeMotor Current", outtakeCode.outtakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        manager.addData("OuttakeMotor Power", outtakeCode.outtakeMotor.getPower());
        manager.addData("OuttakeMotor2 Power", outtakeCode.outtakeMotor.getPower());
        manager.addData("FrontLeft drive power", driveCode.leftFrontDrive.getPower());
        manager.addData("leftBack drive power", driveCode.leftBackDrive.getPower());
        manager.addData("rightFront drive power", driveCode.rightFrontDrive.getPower());
        manager.addData("rightBack drive power", driveCode.rightBackDrive.getPower());*/
        if (PIOuttake.ti == 0){
            manager.addData("Integral/ti", 0);
        }else {
            manager.addData("Integral/ti", outtakeCode.integral / PIOuttake.ti);
        }
        outtakeMotorVelocity = outtakeMotor.getVelocity();
        telemetry.addData("Outtake Motor Velocity", outtakeMotorVelocity);
        telemetry.update();
        robot.update();
        manager.update();
        pastGamepad1.copy(gamepad1);
    }
    void initialize() {
        pastGamepad1 = new Gamepad();
        pastGamepad1.copy(gamepad1);
        try {
            StringBuilder buildStringer = new StringBuilder();
            buildStringer.append(Environment.getExternalStorageDirectory().getPath());
            buildStringer.append("/localizationInfo.txt");
            fileReaderWriter = new FileReadWriter(buildStringer.toString());
        } catch (Exception e) {
            telemetry.addLine(e.getMessage());
            fileReaderWriter = null;
        }

        manager = PanelsTelemetry.INSTANCE.getTelemetry();
        driveCode = new decodeDriveCode(gamepad1, hardwareMap);
        intakeCode = new decodeIntake(hardwareMap, gamepad1);
        outtakeCode = new PIOuttake(hardwareMap, gamepad1, gamepad2);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        runOLime = new LimelightRunner(limelight);
        if (driveCode.drive > 0.25) {
            intakeCode.movingForward = true;
        } else {
            intakeCode.movingForward = false;
        }
        while (!opModeIsActive()) {
            // Get the pipeline (i.e. which field to use), whether we are the red/blue team,
            // and what goal/prism to point at
            setPipeline();
            setColor();
            telemetry.update();
            pastGamepad1.copy(gamepad1);
        }
        robot = Constants.createFollower(hardwareMap);
        try {
            if (fileReaderWriter != null) {
                ArrayList<Double> fromFile = fileReaderWriter.readToDoubles();
                robot.setStartingPose(new Pose(fromFile.get(0), fromFile.get(1), fromFile.get(2)));
            }
        } catch (Exception e) {
            telemetry.addLine(e.getMessage());
        }
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        //pinpoint.setOffsets(6.75, -6.5, DistanceUnit.INCH);
        //pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        //pinpoint.resetPosAndIMU();
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pastGamepad1.copy(gamepad1);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            mainLoop();
        }
    }

}
