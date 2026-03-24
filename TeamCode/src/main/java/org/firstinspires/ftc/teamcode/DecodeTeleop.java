package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drive.decodeDriveCode;
import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.intake.decodeIntake;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@Configurable
@TeleOp(name="DecodeTeleop", group="Enigma")
public class DecodeTeleop extends LinearOpMode {
    public GoBildaPinpointDriver pinpoint;
    decodeDriveCode driveCode;
    decodeIntake intakeCode;
    PIOuttake outtakeCode;
    DcMotorEx outtakeMotor;
    DcMotor outtakeMotor2;
    private TelemetryManager manager;
    Limelight3A limelight;
    LimelightRunner runOLime;
    public int goalTag = 20;
    public double DX = 0;
    public double DIST = 0;
    public double kP_TURN = 0.05;
    public double MAXTURN = 0.75;
    public Pose aprilTagBlue = new Pose(16, 131, /*degrees*/-45);
    public Pose aprilTagRed = new Pose(128, 130, /*degrees*/45);
    public cart2D shootTargetRed = new cart2D(132,136);
    public cart2D shootTargetBlue = new cart2D(12,135);
    public cart2D difference;
    public Pose targetTagPose;
    public double aprilShotDist = 3;
    public double limeDist = 6.5;
    private void setTurnInPlace(double turn) {
        if (Math.abs(turn) > 0.75){
            turn = 0.75 * turn/Math.abs(turn);
        }
        driveCode.leftFrontDrive.setPower(+turn);
        driveCode.leftBackDrive.setPower(+turn);
        driveCode.rightFrontDrive.setPower(-turn);
        driveCode.rightBackDrive.setPower(-turn);
    }

    private void mainLoop() {
        if (gamepad2.x) {
            //bluetag
            goalTag = 20;
            limeDist = limeDist * 1;
            targetTagPose = aprilTagBlue;
            runOLime.switchPipeline(1);//Blue goal pipeline
        } else if (gamepad2.b){
            //redtag
            goalTag = 24;
            limeDist = limeDist * 1;
            targetTagPose = aprilTagRed;
            runOLime.switchPipeline(0);//Red goal pipeline
        }
        if (goalTag == 20) {
            telemetry.addLine("going for blue");
        } else {
            telemetry.addLine("going for red");
        }
        DX = runOLime.getDX(goalTag);
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            if (!Double.isNaN(DX) && Math.abs(DX) <= 1.0) {
                setTurnInPlace(0);
            } else {
                double turn = kP_TURN * DX;
                setTurnInPlace(turn * driveCode.speed);
            }
        } else {
            driveCode.runWheels();
        }
        intakeCode.intake();
        outtakeCode.runUsingPID();

        telemetry.update();
        pinpoint.update();

        telemetry.addData("motor velocity: ", outtakeCode.outtakeMotor.getVelocity());
        manager.addData("MotorVelocity", outtakeMotor.getVelocity());
        manager.addData("Integral", outtakeCode.integral);
        manager.addData("Integral2", outtakeCode.integral2);
        manager.addData("error", outtakeCode.error);
        manager.addData("derivative", outtakeCode.derivative);
        manager.addData("derivative*tD", outtakeCode.derivative * PIOuttake.td);
        telemetry.addData("DX", DX);
        telemetry.addData("DIST", DIST);

        if (PIOuttake.ti == 0){
            manager.addData("Integral/ti", 0);
        }else {
            manager.addData("Integral/ti", outtakeCode.integral/PIOuttake.ti);
        }
        manager.update();
    }
    void initialize() {
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
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        initialize();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "OuttakeRight");
        pinpoint.setOffsets(6.75, -6.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        waitForStart();

        while (opModeIsActive()) {
            mainLoop();
        }
    }

}
