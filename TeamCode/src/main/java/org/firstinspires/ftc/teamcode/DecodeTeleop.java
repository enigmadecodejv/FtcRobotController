package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public int goalTag;
    public double DX = 0;
    public double kP_TURN = 0.02;
    public double MAXTURN = 0.45;

    private void setTurnInPlace(double turn) {
        driveCode.leftFrontDrive.setPower(+turn);
        driveCode.leftBackDrive.setPower(+turn);
        driveCode.rightFrontDrive.setPower(-turn);
        driveCode.rightBackDrive.setPower(-turn);
    }

    private void mainLoop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            // Get all detected AprilTags
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                if (f.getFiducialId() == goalTag) {
                    DX = f.getTargetXDegrees();
                }
            }
        }
        if (gamepad1.x) {
            if (!Double.isNaN(DX) && Math.abs(DX) <= 3) {
                setTurnInPlace(0);
            } else {
                double turn = Range.clip(kP_TURN * DX, -MAXTURN, MAXTURN);
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

        if (PIOuttake.ti == 0){
            manager.addData("Integral/ti", 0);
        }else {
            manager.addData("Integral/ti", outtakeCode.integral/PIOuttake.ti);
        }

    }
    void initialize() {
        manager = PanelsTelemetry.INSTANCE.getTelemetry();
        driveCode = new decodeDriveCode(gamepad1, hardwareMap);
        intakeCode = new decodeIntake(hardwareMap, gamepad1);
        outtakeCode = new PIOuttake(hardwareMap, gamepad1);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
        if (driveCode.drive > 0.25) {
            intakeCode.movingForward = true;
        } else {
            intakeCode.movingForward = false;
        }
        while (!opModeIsActive()) {
            if (gamepad1.x) {
                //bluetag
                goalTag = 20;
                telemetry.addLine("going for blue");
            } else {
                //redtag
                goalTag = 21;
                telemetry.addLine("going for red");
            }
        }
        telemetry.update();
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
