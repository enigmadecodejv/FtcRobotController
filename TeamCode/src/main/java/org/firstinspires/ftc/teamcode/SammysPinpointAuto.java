package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveToAprilTagOmni;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PurpleOrGreen;
import org.firstinspires.ftc.teamcode.vision.aprilTagProcessor;

@Autonomous(name = "PinpointAuto", group = "enigma")
public class SammysPinpointAuto extends LinearOpMode {
    aprilTagProcessor aprilTagProcessor = new aprilTagProcessor(hardwareMap);
    GoBildaPinpointDriver pinpoint;
    double [] targetsXY;
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
   static class artifactsPos {
        public double x;
        public double y;
        PurpleOrGreen purpleOrGreen;
        public artifactsPos(double x, double y, String PurpleorGreen) {
            this.x = x;
            this.y = y;
            if (PurpleorGreen.equals("purple")) {
                this.purpleOrGreen = PurpleOrGreen.purple;
            }else if (PurpleorGreen.equals("green")){
                this.purpleOrGreen = PurpleOrGreen.green;
            }
        }
        public double radius = 5;
    }
    boolean intaking = false;
    double waitForIntake;
    partsOfAuto PartsOfAuto = partsOfAuto.move;
    //define gamepad1
    public Gamepad gamepad1;
    PurpleOrGreen [] motif;
    // declare wheels
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor intake;
    public double speed = 0.75;
    public int quadrant;
    public double artifactMinDisplacement = 16265;
    double artifactDisplacementX;
    double artifactDisplacementY;
    double artifactDisplacementXY;
    int artifactArrayPos;
    int motifPos = 0;
    boolean targetChecked = false;
    int timesAttempted = 1;
    double robotSize = 12;
    double roboLocity;//Make SURE is in inches/second
    double roboLangle;//Make SURE is in degrees/second
    double fps;
    double movementProportions;
    artifactsPos[] artifactArray = {
    //Compared to the team in questions goal

    //blue side
    //nearest
    new artifactsPos(29,84, "purple"),
    new artifactsPos(24,84, "purple"),
    new artifactsPos(19,84, "green"),

    //middle
    new artifactsPos(29,60, "purple"),
    new artifactsPos(24,60, "green"),
    new artifactsPos(19,60, "purple"),

    //farthest
    new artifactsPos(29,36, "green"),
    new artifactsPos(24,36, "purple"),
    new artifactsPos(19,36, "purple"),


    //red side
    //nearest
    new artifactsPos(125,84,"green"),
    new artifactsPos(120, 84, "purple"),
    new artifactsPos(115, 84, "purple"),

    //middle
    new artifactsPos(125, 60, "purple"),
    new artifactsPos(120, 60, "green"),
    new artifactsPos(115, 60, "purple"),

    //farthest
    new artifactsPos(125, 36, "purple"),
    new artifactsPos(120, 36, "purple"),
    new artifactsPos(115, 36, "green")
    };
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
        }else if (displacementX < 0 && displacementY < 0){
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
    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class,"intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        while (!opModeIsActive()) {
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
            motif = aprilTagProcessor.motif();
            if (motif != null) {
                setArtifactTargets(motif[motifPos]);
            }
            if (targetChecked) {
                movementMath();
            }else{
                targetsXY = finalizeTarget(targetX,targetY);
                targetX = targetsXY [0];
                targetY = targetsXY [1];
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
        }
    }
    public void movementMath() {
        double drive = drive();
        double strafe = strafe();
        double turn = turn();
        drive /= drive+strafe;
        strafe /= drive+strafe;
        drive *= movementProportions;
        strafe *= movementProportions;
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
    public double movementProportions(double displacement, double angleAdd){
        double moveTime = ((displacement - 1)/roboLocity) + (Math.log(0.01)/Math.log(1-roboLocity));
        double turnTime = (((angleAdd/Math.PI) - 1)/roboLangle) + (Math.log(0.01)/Math.log(1-roboLangle));
        if (angleAdd == 0){
            turnTime = 0;
        }
        if (displacement == 0){
            moveTime = 0;
        }
        //Returns what drive and strafe should add up to
        return moveTime/(moveTime+turnTime);
    }
    public void setArtifactTargets(PurpleOrGreen color){
        for (int i = 0; i < artifactArray.length - 1; i++){
            if (artifactArray[i].purpleOrGreen == color){
                artifactDisplacementX = artifactArray[i].x - pinpoint.getPosX(DistanceUnit.INCH);
                artifactDisplacementY = artifactArray[i].y - pinpoint.getPosY(DistanceUnit.INCH);
                artifactDisplacementXY = Math.sqrt(Math.pow(artifactDisplacementX,2) + Math.pow(artifactDisplacementY,2));
                if (artifactDisplacementXY < artifactMinDisplacement) {
                    artifactMinDisplacement = artifactDisplacementXY;
                    artifactArrayPos = i;
                }
            }
        }
        targetX = artifactArray[artifactArrayPos].x;
        targetY = artifactArray[artifactArrayPos].y;
        targetChecked = false;
    }
    public int checkIntersection(double targetX, double targetY){
        //returns the position in artifactArray of the first artifact (in artifactArray) the line segment from robot to target intersects with
        double slope = (targetX - pinpoint.getPosX(DistanceUnit.INCH))/(targetY - pinpoint.getPosY(DistanceUnit.INCH));
        double linePosY = (pinpoint.getPosY(DistanceUnit.INCH) - (slope * pinpoint.getPosX(DistanceUnit.INCH)));
        double[] intersectionX;
        double coeffXpwr2;
        double coeffXpwr1;
        double coeffXpwr0;
        int firstIntersection = 16265;
        double[] interval;
        for (int j = 0; j < artifactArray.length - 1; j++){
            coeffXpwr2 = Math.pow(slope,2) + 1;
            coeffXpwr1 = 2*((slope*(linePosY - artifactArray[j].y)) - artifactArray[j].x);
            coeffXpwr0 = Math.pow(linePosY - artifactArray[j].y,2) - Math.pow(artifactArray[j].radius,2);
            intersectionX = new double[]{(-coeffXpwr1 + Math.sqrt(Math.pow(coeffXpwr1, 2) - 4 * coeffXpwr2 * coeffXpwr0)) / (2 * coeffXpwr2), (-coeffXpwr1 - Math.sqrt(Math.pow(coeffXpwr1, 2) - 4 * coeffXpwr2 * coeffXpwr0)) / (2 * coeffXpwr2)};
            if (targetX > pinpoint.getPosX(DistanceUnit.INCH)){
                interval = new double[] {pinpoint.getPosX(DistanceUnit.INCH), targetX};
            }else if (artifactArray[j].x < pinpoint.getPosX(DistanceUnit.INCH)){
                interval = new double[] {targetX, pinpoint.getPosX(DistanceUnit.INCH)};
            }else{
                interval = new double[]{0,0};
            }
            if (interval[0] <= intersectionX[0] && intersectionX[0] <= interval[1] || interval[0] <= intersectionX[1] && intersectionX[1] <= interval[1]){
                firstIntersection = j;
                break;
            }
        }
        return firstIntersection;
    }
    public double[] finalizeTarget(double targetX, double targetY){
        double [] newTarget;
        int intersect = checkIntersection(targetX,targetY);
        if (intersect != 16265){
            if (timesAttempted%2 != 0){
                if (targetX - pinpoint.getPosX(DistanceUnit.INCH) > targetY - pinpoint.getPosY(DistanceUnit.INCH)){
                    newTarget = new double [] {artifactArray[intersect].x,artifactArray[intersect].y + (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1)};
                }else{
                    newTarget = new double [] {artifactArray[intersect].x + (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1), artifactArray[intersect].y};
                }
            }else{
                if (targetX - pinpoint.getPosX(DistanceUnit.INCH) > targetY - pinpoint.getPosY(DistanceUnit.INCH)){
                    newTarget = new double [] {artifactArray[intersect].x,artifactArray[intersect].y - (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1)};
                }else{
                    newTarget = new double [] {artifactArray[intersect].x - (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1), artifactArray[intersect].y};
                }
            }
            targetChecked = false;
        }else{
            newTarget = new double[] {targetX, targetY};
            targetChecked = true;
            drive();
            turn();
            strafe();
            movementProportions = movementProportions(displacement,angleAdd);
        }
        return newTarget;
    }
    public void intake () {
        intake.setPower(1);
    }
}
