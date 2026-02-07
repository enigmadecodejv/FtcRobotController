package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TargetMoving2 extends LinearOpMode {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public GoBildaPinpointDriver pinpoint;
    public double targetX = 0;
    public double targetY = 0;
    public double targetAngle;
    public boolean reachedPos = false;
    public double finalAngle;
    public double rawAngle;
    public double displaceX;
    public double displaceY;
    public double speed = 0.75;
    //drive kinematics input
    public void runToPos(
            double targetX,
            double targetY,
            double targetAngle,
            double PosX,
            double PosY,
            double roboAngle,
            boolean reachedPos
            ){
        double drive = 0;
        double turn = 0;
        displaceX = targetX - PosX;
        displaceY = targetY - PosY;
        rawAngle = Math.atan(displaceY/displaceX);
        if (displaceX < 0){
            finalAngle += Math.PI;
        }else{
            finalAngle = rawAngle;
        }
        if (roboAngle != finalAngle && !reachedPos){
            turn = finalAngle - roboAngle;
        }else if (PosX != targetX || PosY != targetY){
            drive = Math.pow(Math.pow(displaceX,2) + Math.pow(displaceY,2), 1/2);
        }else if (roboAngle != targetAngle){
            turn = targetAngle - roboAngle;
        }
        double leftFront = drive + turn;
        double leftBack = drive + turn;
        double rightFront = drive - turn;
        double rightBack = drive - turn;

        // Normalize the values so neither exceed +/- 1.0

        double maxFront = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        double maxBack = Math.max(Math.abs(leftBack),Math.abs(rightBack));
        double max = Math.max(maxFront,maxBack);
        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        // Output the safe vales to the motor drives.
        leftFrontDrive.setPower(leftFront * speed);
        leftBackDrive.setPower(leftBack * speed);
        rightFrontDrive.setPower(rightFront * speed);
        rightBackDrive.setPower(rightBack * speed);
    }
    public void runOpMode(){
        leftFrontDrive = hardwareMap.get(DcMotor .class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        pinpoint.setOffsets(2, 2.5, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        waitForStart();
        while(opModeIsActive()){
            runToPos(targetX,targetY,targetAngle,pinpoint.getPosX(DistanceUnit.INCH),pinpoint.getPosY(DistanceUnit.INCH),pinpoint.getHeading(AngleUnit.RADIANS),reachedPos);
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY){
                reachedPos = true;
            }
        }
    }
}
