package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PurpleOrGreen;
import org.firstinspires.ftc.teamcode.vision.aprilTagProcessor;

@Autonomous(name = "PinpointAuto", group = "enigma")
public class SammysPinpointAuto extends LinearOpMode {
    generalMethodsAuto methods = new generalMethodsAuto();
    aprilTagProcessor AprilTagProcessor;
    targetMoveTest targetMove;
    targetingSystem targeting;
    distanceOuttake outtakeCode;
    public GoBildaPinpointDriver pinpoint;
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
    int motifPos = 0;
    boolean starting = true;
    boolean dontMove = false;
    boolean isRed;
    double [] shotPointsRed = {80,90,125,130};
    double [] shotPointsBlue = {65,90,20,130};
    PurpleOrGreen [] motif;
    double redGoalX;
    double redGoalY;
    double blueGoalX;
    double blueGoalY;
    double artifactsInRobot = 3;
    double [] finalPos;
    double servoTimer;
    double servoTimer0;
    public void runOpMode() {
        AprilTagProcessor = new aprilTagProcessor(hardwareMap);
        targetMove = new targetMoveTest(/*hardwareMap*/);//todo fix targetMoveTest to work in another opMode when testing is finished
        targeting = new targetingSystem(/*hardwareMap*/);//todo fix targetingSystem to work in another opMode when testing is finished
        outtakeCode = new distanceOuttake();
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
                if (PartsOfAuto == partsOfAuto.move2 && pinpoint.getPosX(DistanceUnit.INCH) + RCC[0] == targetX && pinpoint.getPosY(DistanceUnit.INCH) + RCC[1] == targetY) {
                    PartsOfAuto = partsOfAuto.intake;
                    intaking = true;
                    waitForIntake = System.currentTimeMillis();
                } else if (PartsOfAuto == partsOfAuto.intake && System.currentTimeMillis() > waitForIntake + 300) {
                    artifactsInRobot++;
                    if (artifactsInRobot < 3){
                        PartsOfAuto = partsOfAuto.move1;
                    }else {
                        intaking = false;
                        PartsOfAuto = partsOfAuto.move2;
                    }
                } else if (PartsOfAuto == partsOfAuto.move2 && pinpoint.getPosX(DistanceUnit.INCH) + RCC[0] == targetX && pinpoint.getPosY(DistanceUnit.INCH) + RCC[1] == targetY && targetsXY == finalPos) {
                    PartsOfAuto = partsOfAuto.outtake;
                    waitForIntake = System.currentTimeMillis();
                }else if (PartsOfAuto == partsOfAuto.outtake && artifactsInRobot == 0){
                    PartsOfAuto = partsOfAuto.move1;
                }
                if (intaking) {
                    intake();
                }
                pinpoint.update();
                motif = AprilTagProcessor.findmotif();
                if (aprilTagProcessor.motif[motifPos] != null && PartsOfAuto == partsOfAuto.move1) {
                    targetsXY = targeting.setArtifactTargets(aprilTagProcessor.motif[motifPos]);
                    finalPos = targetsXY;
                    targetX = targetsXY[0];
                    targetY = targetsXY[1];
                } else if (PartsOfAuto == partsOfAuto.move2){
                    double [] point1;
                    double [] headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
                    double PosX = pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0];
                    double PosY = pinpoint.getPosY(DistanceUnit.INCH) + headingChange[1];
                    if (isRed){
                        point1 = targeting.findClosestShootPoint(new double [] {shotPointsRed[0], shotPointsRed[1]}, new double []{shotPointsRed[2],shotPointsRed[3]}, new double []{shotPointsRed[4],shotPointsRed[5]}, PosX, PosY);
                    }else{
                        point1 = targeting.findClosestShootPoint(new double [] {shotPointsBlue[0], shotPointsBlue[1]}, new double []{shotPointsBlue[2],shotPointsBlue[3]}, new double []{shotPointsBlue[4],shotPointsBlue[5]}, PosX, PosY);
                    }
                    targetX = point1[0];
                    targetY = point1[1];
                    targetsXY = point1;
                    finalPos = point1;
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
                    if (servoTimer > System.currentTimeMillis() + 100) {
                        outtakeServo.setPosition(0.88);
                        artifactsInRobot--;
                        servoTimer0 = System.currentTimeMillis();
                    }else if (servoTimer0 > System.currentTimeMillis() + 100){
                        outtakeServo.setPosition(0.98);
                        servoTimer = System.currentTimeMillis();
                        intake();
                    }
                }
                if (targeting.targetChecked) {
                    targetMove.movementMath();
                } else if (PartsOfAuto == partsOfAuto.move1 || PartsOfAuto == partsOfAuto.move2){
                    targetsXY = targeting.finalizeTarget(targetX, targetY, pinpoint.getHeading(AngleUnit.RADIANS));
                    targetX = targetsXY[0];
                    targetY = targetsXY[1];
                }
                double PosX = pinpoint.getPosX(DistanceUnit.INCH) + RCC[0];
                double PosY = pinpoint.getPosX(DistanceUnit.INCH) + RCC[1];
                double shootVelocity;
                if (isRed) {
                    shootVelocity = outtakeCode.getVelocityShot(new double[]{PosX, PosY}, new double []{redGoalX,redGoalY});
                }else{
                    shootVelocity = outtakeCode.getVelocityShot(new double []{PosX,PosY}, new double []{blueGoalX,blueGoalY});
                }
                double power = outtakeCode.autoOuttake(shootVelocity);
                outtakeMotor.setPower(power);
                outtakeMotor2.setPower(power);
            }
        }
    }
    public void intake () {
        intake.setPower(1);
    }
}
