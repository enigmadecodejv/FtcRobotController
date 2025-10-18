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
        move0,
        intake0,
        move1
    }
    boolean intakeing = false;
    double waitForIntake;
    partsOfAuto PartsOfAuto = partsOfAuto.move0;
    //define gamepad1
    public Gamepad gamepad1;

    // declare wheels
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public double speed = 0.75;
    public double drive() {
        displacementX = targetX - pinpoint.getPosX(DistanceUnit.INCH);
        displacementY = targetY - pinpoint.getPosY(DistanceUnit.INCH);
        displacement = Math.pow(Math.pow(displacementX, 2) + Math.pow(displacementY, 2), 0.5);
        angle = Math.atan(displacementY / displacementX);
        angleAdd = Math.abs(angle) + Math.abs(pinpoint.getHeading(AngleUnit.RADIANS));
        return Math.cos(angleAdd) * displacement;
    }
    public double strafe() {
        return Math.sin(angleAdd) * displacement;
    }
    public double turn() {
      return angleAdd * 180/Math.PI;
    }
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        waitForStart();
        while (opModeIsActive()) {
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY && PartsOfAuto == partsOfAuto.move0) {
            PartsOfAuto = partsOfAuto.intake0;
            intakeing = true;
            waitForIntake = System.currentTimeMillis();
            } else if (PartsOfAuto == partsOfAuto.intake0 && System.currentTimeMillis() > waitForIntake + 300) {
                intakeing = false;
                PartsOfAuto = partsOfAuto.move1;
            }
            if (intakeing) {
                intake();
            }
            pinpoint.update();
            movementMath();
        }
    }
    public void movementMath() {
        double leftFront = drive() + turn() + strafe();
        double leftBack = drive() + turn() - strafe();
        double rightFront = drive() - turn() - strafe();
        double rightBack = drive() - turn() + strafe();

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
}
