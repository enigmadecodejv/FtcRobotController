package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "moveTest", group = "enigma")
public class targetMoveTest extends LinearOpMode {
    public double displacementX;
    public double displacementY;
    public double displacement;
    public double targetX;
    public double targetY;
    public double angle;
    public double angleAdd;
    public int quadrant;
    public double speed = 0.75;
    public DcMotor leftFrontDrive = hardwareMap.get(DcMotor .class, "FrontLeft");
    public DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
    public DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
    public DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");
    public GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
    public double drive() {
        //find x and y position compared to robot (x - xnot)
        displacementX = targetX - pinpoint.getPosX(DistanceUnit.INCH);
        displacementY = targetY - pinpoint.getPosY(DistanceUnit.INCH);
        //find x and y positon in polar coordinates (r@theta)
        displacement = Math.pow(Math.pow(displacementX, 2) + Math.pow(displacementY, 2), 0.5);
        angle = Math.atan(displacementY / displacementX);
        if (angle > Math.PI * 2){
            angle = angle - Math.PI * 2;
        }else if (angle < 0){
            angle = angle + Math.PI;
        }
        if (displacementX > 0 && displacementY > 0){
            if (angle > Math.PI/2){
                angle = angle + Math.PI;
            }
        }else if (displacementX < 0 && displacementY > 0){
            if (angle < Math.PI/2 || angle > Math.PI){
                angle += Math.PI;
            }
        }else if (displacementX < 0 && displacementY < 0){
            if (angle < Math.PI || angle > 3 * Math.PI/2){
                angle -= Math.PI;
            }else if (displacementX > 0 && displacementY < 0){
                if (angle < 3*Math.PI/2){
                    angle -= Math.PI;
                }
            }
        }
        //find a theta based on the robot's rotation and angle
        angleAdd = angle - pinpoint.getHeading(AngleUnit.RADIANS);
        //use that theta to find polar coordinates
        double drive;
        //find quadrant
        if (displacementX > 0 && displacementY > 0){
            quadrant = 1;
        }else if (displacementX < 0 && displacementY > 0){
            quadrant = 2;
        }else if (displacementX < 0 && displacementY < 0){
            quadrant = 3;
        }else if (displacementX > 0 && displacementY < 0){
            quadrant = 4;
        }
        //use that quadrant to find the x coordinate based on sin and cos
        if (quadrant%2 == 0){
            drive = Math.sin(angleAdd) * displacement;
        }else {
            drive = Math.cos(angleAdd) * displacement;
        }
        return drive;
    }
    public double strafe() {
        //use that quadrant to find the y coordinate based on sin and cos
        double strafe;
        if (quadrant%2 == 0){
            strafe = Math.cos(angleAdd) * displacement;
        }else {
            strafe = Math.sin(angleAdd) * displacement;
        }
        return strafe;
    }
    public double turn() {
        //find how much the robot needs to turn
        return angleAdd * 180/Math.PI;

    }
    public void movementMath() {
        double drive = drive();
        double strafe = strafe();
        double turn = turn();
        //I HAVE DONE THEFT!
        double leftFront = drive + turn/180 + strafe;
        double leftBack = drive + turn/180 - strafe;
        double rightFront = drive - turn/180 - strafe;
        double rightBack = drive - turn/180 + strafe;

        // Normalize the values so neither exceed +/- 1.0

        double maxFront = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        double maxBack = Math.max(Math.abs(leftBack),Math.abs(rightBack));
        double max = Math.max(maxFront,maxBack);
        if (max > 1.0)
        {
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
    public void runOpMode (){
        movementMath();
        if (gamepad1.a){
            targetX += 1;
        }
        if (gamepad1.b){
            targetY += 1;
        }
        if (gamepad1.x){
            targetX -= 1;
        }
        if (gamepad1.y){
            targetY -= 1;
        }
    }
}
