package org.firstinspires.ftc.teamcode.Teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Drive.NaNTurnBehavior;
import org.firstinspires.ftc.teamcode.Drive.decodeDriveCode;
import org.firstinspires.ftc.teamcode.LimelightRunner;
import org.firstinspires.ftc.teamcode.Outtake.PIOuttake;
import org.firstinspires.ftc.teamcode.intake.decodeIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name="DecodeTeleopBackup", group="Enigma")
public class DecodeTeleopBackup extends LinearOpMode {
    //public GoBildaPinpointDriver pinpoint;
    public PathChain path;
    public Follower robot;
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
    public static double kP_TURN = 0.03;
    public static double tI_TURN = 960;
    public double integralTurn = 0;
    public double MAXTURN = 0.75;
    //public Pose aprilTagBlue = new Pose(16, 131, /*degrees*/-45);
    //public Pose aprilTagRed = new Pose(128, 130, /*degrees*/45);
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
    private void mainLoop() {
        if (gamepad2.left_trigger > 0.25){
            Pose limelightResult = runOLime.getBotPose();
            if (limelightResult != null){
                localizePinpoint(limelightResult);
            }
        }
        if (gamepad2.x) {
            //bluetag
            goalTag = 20;
            runOLime.switchPipeline(1);//Blue goal pipeline is 1
        } else if (gamepad2.b){
            //redtag
            goalTag = 24;
            runOLime.switchPipeline(0);//Red goal pipeline is 0
        }
        if (goalTag == 20) {
            telemetry.addLine("going for blue");
        } else {
            telemetry.addLine("going for red");
        }
        DX = runOLime.getDX(goalTag);
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
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

        telemetry.addData("motor velocity: ", outtakeCode.outtakeMotor.getVelocity());
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
        }
        telemetry.addData("DX", DX);
        manager.addData("DX", DX);
        telemetry.addData("DIST", DIST);
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
        telemetry.addData("OuttakeMotor Current", outtakeCode.outtakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("OuttakeMotor Power", outtakeCode.outtakeMotor.getPower());
        telemetry.addData("OuttakeMotor2 Power", outtakeCode.outtakeMotor.getPower());
        /*telemetry.addData("FrontLeft drive power", driveCode.leftFrontDrive.getPower());
        telemetry.addData("leftBack drive power", driveCode.leftBackDrive.getPower());
        telemetry.addData("rightFront drive power", driveCode.rightFrontDrive.getPower());
        telemetry.addData("rightBack drive power", driveCode.rightBackDrive.getPower());
        */manager.addData("OuttakeMotor Current", outtakeCode.outtakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        manager.addData("OuttakeMotor Power", outtakeCode.outtakeMotor.getPower());
        manager.addData("OuttakeMotor2 Power", outtakeCode.outtakeMotor.getPower());
        /*manager.addData("FrontLeft drive power", driveCode.leftFrontDrive.getPower());
        manager.addData("leftBack drive power", driveCode.leftBackDrive.getPower());
        manager.addData("rightFront drive power", driveCode.rightFrontDrive.getPower());
        manager.addData("rightBack drive power", driveCode.rightBackDrive.getPower());
*/
        if (PIOuttake.ti == 0){
            manager.addData("Integral/ti", 0);
        }else {
            manager.addData("Integral/ti", outtakeCode.integral / PIOuttake.ti);
        }
        telemetry.update();
        robot.update();
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
        robot = Constants.createFollower(hardwareMap);

        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        //pinpoint.setOffsets(6.75, -6.5, DistanceUnit.INCH);
        //pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        //pinpoint.resetPosAndIMU();
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
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
