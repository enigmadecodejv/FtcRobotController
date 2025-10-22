package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SammysPinpointAuto extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    double targetX;
    double targetY;
    double displacementX;
    double displacementY;
    double displacement;
    double angle;
    double angleAdd;
    enum partsOfAuto{
        move,
        intake,
        rotate,
        outtake
    }
    enum motif{
        ppg,
        pgp,
        gpp
    }
    boolean intaking = false;
    double waitForIntake;
    partsOfAuto PartsOfAuto = partsOfAuto.move;
    //define gamepad1
    public Gamepad gamepad1;
    motif Motif;
    // declare wheels
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor intake;
    public double speed = 0.75;
    public int quadrant;
    artifactsPos[] artifactArray = {new artifactsPos(12,6, "green"), new artifactsPos(12,6, "blue")};
    public double hi = artifactArray[1].x;
    public double drive() {
        displacementX = targetX - pinpoint.getPosX(DistanceUnit.INCH);
        displacementY = targetY - pinpoint.getPosY(DistanceUnit.INCH);
        displacement = Math.pow(Math.pow(displacementX, 2) + Math.pow(displacementY, 2), 0.5);
        angle = Math.atan(displacementY / displacementX);
        angleAdd = angle - pinpoint.getHeading(AngleUnit.RADIANS);
        double drive;
        if (displacementX > 0 && displacementY > 0){
            quadrant = 1;
        }else if (displacementX < 0 && displacementY > 0){
            quadrant = 2;
        }else if (displacementX < 0 && displacementY < 0){
            quadrant = 3;
        }else if (displacementX > 0 && displacementY < 0){
            quadrant = 4;
        }
        if (quadrant%2 == 0){
            drive = Math.sin(angleAdd) * displacement;
        }else {
            drive = Math.cos(angleAdd) * displacement;
        }
            return drive;
    }
    public double strafe() {
        double strafe;
        if (quadrant%2 == 0){
            strafe = Math.cos(angleAdd) * displacement;
        }else {
            strafe = Math.sin(angleAdd) * displacement;
        }
        return strafe;
    }
    public double turn() {
      return angleAdd * 180/Math.PI;
    }
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class,"intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        while (!opModeIsActive()) {
            if (gamepad1.a){
                Motif = motif.gpp;
            }else if (gamepad1.b){
                Motif = motif.pgp;
            }else if (gamepad1.x){
                Motif = motif.ppg;
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY && PartsOfAuto == partsOfAuto.move) {
            PartsOfAuto = partsOfAuto.intake;
            intaking = true;
            waitForIntake = System.currentTimeMillis();
            } else if (PartsOfAuto == partsOfAuto.intake && System.currentTimeMillis() > waitForIntake + 300) {
                intaking = false;
                PartsOfAuto = partsOfAuto.move;
            }
            if (intaking) {
                intake();
            }
            pinpoint.update();
            movementMath();
        }
    }
    public void movementMath() {
        double leftFront = drive() + turn()/180 + strafe();
        double leftBack = drive() + turn()/180 - strafe();
        double rightFront = drive() - turn()/180 - strafe();
        double rightBack = drive() - turn()/180 + strafe();

        // Normalize the values so neither exceed +/- 1.0

        double maxFront = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        double maxBack = Math.max(Math.abs(leftBack),Math.abs(rightBack));
        double max = Math.max(maxFront,maxBack);
        if (max > 1.0)
        {
            leftFront /= max;
            rightFront /= max;
        }



        // Output the safe vales to the motor drives.
        leftFrontDrive.setPower(leftFront * speed);
        leftBackDrive.setPower(leftBack * speed);
        rightFrontDrive.setPower(rightFront * speed);
        rightBackDrive.setPower(rightBack * speed);
    }
    public  void intake () {
        intake.setPower(1);
    }
}
