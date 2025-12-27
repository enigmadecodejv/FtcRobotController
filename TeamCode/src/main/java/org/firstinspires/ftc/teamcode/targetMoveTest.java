package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "moveTest", group = "enigma")
public class targetMoveTest extends LinearOpMode {
    public generalMethodsAuto methods = new generalMethodsAuto();
    public double targetX = 15;
    public double targetY = 15;
    public int quadrant;
    public double speed = 0.75;
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public GoBildaPinpointDriver pinpoint;
    public double drive;
    public double strafe;
    public double turn;
    public double [] headingChange;
    public double [] moveVariables;
    public double roboLocity; //make SURE is in inches/second
    public double roboLangle; //make SURE is in degrees/second
    public double drive() {
        headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
        moveVariables = methods.findDisplacementAndAngleAdd(
                pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0],
                pinpoint.getPosY(DistanceUnit.INCH) + headingChange[1],
                pinpoint.getHeading(AngleUnit.RADIANS),
                targetX,
                targetY);
        //use that theta to find polar coordinates
        double drive;
        //find quadrant
        if (moveVariables[1] < Math.PI/2){
            quadrant = 1;
        }else if (moveVariables[1] >= Math.PI/2 && moveVariables[1] < Math.PI){
            quadrant = 2;
        }else if (moveVariables[1] >= Math.PI && moveVariables[1] < 3*Math.PI/2){
            quadrant = 3;
        }else if (moveVariables[1] >= 3*Math.PI/2){
            quadrant = 4;
        }
        //use that quadrant to find the x coordinate based on sin and cos
        if (quadrant%2 == 0){
            drive = Math.sin(moveVariables[1]) * moveVariables[0];
        }else {
            drive = Math.cos(moveVariables[1]) * moveVariables[0];
        }
        return drive;
    }
    public double strafe() {
        //use that quadrant to find the y coordinate based on sin and cos
        double strafe;
        if (quadrant%2 == 0){
            strafe = Math.cos(moveVariables[1]) * moveVariables[0];
        }else {
            strafe = Math.sin(moveVariables[1]) * moveVariables[0];
        }
        return strafe;
    }
    public double turn() {
        //find how much the robot needs to turn
        return moveVariables[1] * 180/Math.PI;

    }
    public double movementProportions(double displacement, double angleAdd) {
        double moveTime = Math.pow(-1,0.5);
        double turnTime = Math.pow(-1,0.5);
        if ((-displacement + Math.pow(Math.pow(displacement,2)-4*roboLocity*displacement,0.5))/2 > 0){
            moveTime = (-displacement + Math.pow(Math.pow(displacement,2)-4*roboLocity*displacement,0.5))/2;
        }else if ((-displacement - Math.pow(Math.pow(displacement,2)-4*roboLocity*displacement,0.5))/2 > 0){
            moveTime = (-displacement - Math.pow(Math.pow(displacement,2)-4*roboLocity*displacement,0.5))/2;
        }else{
            telemetry.addData("NOOOOO!", "Help please!");
        }
        if ((-displacement + Math.pow(Math.pow(displacement,2)-4*roboLangle*displacement,0.5))/2 > 0){
            turnTime = (-displacement + Math.pow(Math.pow(displacement,2)-4*roboLangle*displacement,0.5))/2;
        }else if ((-displacement - Math.pow(Math.pow(displacement,2)-4*roboLangle*displacement,0.5))/2 > 0){
            turnTime = (-displacement - Math.pow(Math.pow(displacement,2)-4*roboLangle*displacement,0.5))/2;
        }else{
            telemetry.addData("NOOOOO!", "Help please!");
        }
        //Returns what drive and strafe should add up to
        return moveTime/(moveTime+turnTime);
    }
    public void movementMath() {
        drive = drive();
        strafe = strafe();
        turn = turn();
        double sumDriveStrafe = movementProportions(moveVariables[0], moveVariables[1]) * turn();
        drive = drive * sumDriveStrafe/(drive + strafe);
        strafe = strafe * sumDriveStrafe/(drive + strafe);
        //I HAVE DONE THEFT!
        double leftFront = drive + turn/180 + strafe;
        double leftBack = drive + turn/180 - strafe;
        double rightFront = drive - turn/180 - strafe;
        double rightBack = drive - turn/180 + strafe;

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
        telemetry.addData("Drive: ", drive);
        telemetry.addData("Strafe: ", strafe);
        telemetry.addData("PinpointHeading: ", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.addData("PinpointX: ", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("PinpointY: ", pinpoint.getPosY(DistanceUnit.INCH));


        // Output the safe vales to the motor drives.
        leftFrontDrive.setPower(leftFront * speed);
        leftBackDrive.setPower(leftBack * speed);
        rightFrontDrive.setPower(rightFront * speed);
        rightBackDrive.setPower(rightBack * speed);
    }
    public void runOpMode () {
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
        while (opModeIsActive()) {
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
        if (gamepad1.y) {
            targetY -= 1;
        }
        telemetry.update();
        pinpoint.update();
        }
    }
}
