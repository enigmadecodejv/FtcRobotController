package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PurpleOrGreen;
import org.firstinspires.ftc.teamcode.vision.aprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "PinpointAuto", group = "enigma")
public class SammysPinpointAuto extends LinearOpMode {
    generalMethodsAuto methods = new generalMethodsAuto();
    aprilTagProcessor AprilTagProcessor;
    targetMoveTest targetMove;
    targetingSystem targeting;
    public GoBildaPinpointDriver pinpoint;
    public double RCCX = 0.42;//robot center correction X
    public double RCCY = -4.375;//robot center correction Y
    double [] targetsXY;
    double targetX;
    double targetY;
    enum partsOfAuto{
        move1,
        intake,
        move2,
        rotate,
        outtake
    }
    boolean intaking = false;
    double waitForIntake;
    partsOfAuto PartsOfAuto = partsOfAuto.move2;
    //define gamepad1
    public Gamepad gamepad1;
    // declare wheels
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor intake;
    public DcMotor outtakeMotor;
    public DcMotor outtakeMotor2;
    public Servo outtakeServo;
    public double artifactMinDisplacement = 16265;
    double artifactDisplacementX;
    double artifactDisplacementY;
    double artifactDisplacementXY;
    int artifactArrayPos;
    int motifPos = 0;
    boolean targetChecked = false;
    boolean starting = true;
    boolean dontMove = false;
    boolean isRed;
    int timesAttempted = 1;
    double robotSize = 12;
    double [] shotPointsRed = {80,90,125,130};
    double [] shotPointsBlue = {65,90,20,130};
    double roboLocity;//Make SURE is in inches/second
    double roboLangle;//Make SURE is in degrees/second
    double movementProportions;
    PurpleOrGreen [] motif;
    double redGoalX;
    double redGoalY;
    double blueGoalX;
    double blueGoalY;
    public void runOpMode() {
        AprilTagProcessor = new aprilTagProcessor(hardwareMap);
        targetMove = new targetMoveTest(/*hardwareMap*/);//todo fix targetMoveTest to work in another opMode when testing is finished
        targeting = new targetingSystem(/*hardwareMap*/);//todo fix targetingSystem to work in another opMode when testing is finished
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor = hardwareMap.get(DcMotor.class, "ShooterRight");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "ShooterLeft");
        outtakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo = hardwareMap.get(Servo.class, "Feeder");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        pinpoint.setOffsets(2, 2.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        AprilTagProcessor.initialize();

        while (!opModeIsActive()) {
            if (gamepad1.a){
                pinpoint.setHeading(Math.PI, AngleUnit.RADIANS);
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            if (starting) {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (AprilTagProcessor.findmotif() == null) {
                    leftFrontDrive.setPower(0.5);
                    leftBackDrive.setPower(0.5);
                    rightFrontDrive.setPower(-0.5);
                    rightBackDrive.setPower(-0.5);
                } else {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    pinpoint.setPosX((-AprilTagProcessor.aprilCoords()) * Math.sin(pinpoint.getHeading(AngleUnit.RADIANS)), DistanceUnit.INCH);
                    pinpoint.setPosY((-AprilTagProcessor.aprilCoords()) * Math.cos(pinpoint.getHeading(AngleUnit.RADIANS)), DistanceUnit.INCH);
                    starting = false;
                }
            }else {
                double [] RCC = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
                if (pinpoint.getPosX(DistanceUnit.INCH) + RCC[0] == targetX && pinpoint.getPosY(DistanceUnit.INCH) + RCC[1] == targetY && PartsOfAuto == partsOfAuto.move1) {
                    PartsOfAuto = partsOfAuto.intake;
                    intaking = true;
                    waitForIntake = System.currentTimeMillis();
                } else if (PartsOfAuto == partsOfAuto.intake && System.currentTimeMillis() > waitForIntake + 300) {
                    intaking = false;
                    PartsOfAuto = partsOfAuto.move2;
                }
                if (intaking) {
                    intake();
                }
                pinpoint.update();
                motif = AprilTagProcessor.findmotif();
                if (aprilTagProcessor.motif[motifPos] != null && PartsOfAuto == partsOfAuto.move1) {
                    targeting.setArtifactTargets(aprilTagProcessor.motif[motifPos]);
                } else if (PartsOfAuto == partsOfAuto.move2){
                    if (isRed){
                    }else{

                    }
                }else if (PartsOfAuto == partsOfAuto.rotate){
                    if (isRed) {
                        targetX = redGoalX;
                        targetY = redGoalY;
                        targetsXY = new double[]{redGoalX, redGoalY};
                    }else {
                        targetX = blueGoalX;
                        targetY = blueGoalY;
                        targetsXY = new double[]{blueGoalX, blueGoalY};
                    }
                        dontMove = true;
                }else if (PartsOfAuto == partsOfAuto.outtake){
                    
                }
                if (targetChecked) {
                    targetMove.movementMath();
                } else if (PartsOfAuto == partsOfAuto.move1 || PartsOfAuto == partsOfAuto.move2){
                    targetsXY = targeting.finalizeTarget(targetX, targetY);
                    targetX = targetsXY[0];
                    targetY = targetsXY[1];
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                }
            }
        }
    }
/*    public double [] findClosestShootPoint(){
        if (isRed){

        }else{

        }
    }*/
    public void intake () {
        intake.setPower(1);
    }
}
