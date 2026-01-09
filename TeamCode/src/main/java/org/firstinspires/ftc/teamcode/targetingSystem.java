package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PurpleOrGreen;

import java.util.Arrays;

@Autonomous(name = "targetingSystem", group = "enigma")
public class targetingSystem extends LinearOpMode {
    public generalMethodsAuto methods = new generalMethodsAuto();
    public targetMoveTest targetMove;
    public GoBildaPinpointDriver pinpoint;
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    PurpleOrGreen color = PurpleOrGreen.green;
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
    double artifactDisplacementX;
    double artifactDisplacementY;
    double artifactDisplacementXY;
    public double artifactMinDisplacement = 16265;
    int artifactArrayPos;
    double targetX;
    double targetY;
    boolean targetChecked = false;
    double timesAttempted = 1;
    double robotSize = 12;
    public double [] headingChange;
    public double [] targetsXY;
    public boolean targetsSet = false;
    public boolean reachedArtifact = false;
    double [] shotPointsRed = {80,90,125,130,82,10};
    double [] shotPointsBlue = {65,90,20,130,62,10};
    public boolean isRed = true;
    public double [] setArtifactTargets(PurpleOrGreen color){
        headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
        for (int i = 0; i < artifactArray.length - 1; i++){
            if (artifactArray[i].purpleOrGreen == color){
                artifactDisplacementX = artifactArray[i].x - pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0];
                artifactDisplacementY = artifactArray[i].y - pinpoint.getPosY(DistanceUnit.INCH) + headingChange[1];
                artifactDisplacementXY = Math.sqrt(Math.pow(artifactDisplacementX,2) + Math.pow(artifactDisplacementY,2));
                if (artifactDisplacementXY < artifactMinDisplacement) {
                    artifactMinDisplacement = artifactDisplacementXY;
                    artifactArrayPos = i;
                }
            }
        }
        targetChecked = false;
        return new double [] {artifactArray[artifactArrayPos].x, artifactArray[artifactArrayPos].y};
    }
    public int checkIntersection(double targetX, double targetY){
        headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
        //returns the position in artifactArray of the first artifact (in artifactArray) the line segment from robot to target intersects with
        double slope = (targetX - pinpoint.getPosX(DistanceUnit.INCH) - headingChange[0])/(targetY - pinpoint.getPosY(DistanceUnit.INCH) - headingChange[1]);
        double linePosY = pinpoint.getPosY(DistanceUnit.INCH) + headingChange[0] - (slope * (pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0]));
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
            if (targetX > pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0]){
                interval = new double[] {pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0], targetX};
            }else if (artifactArray[j].x < pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0]){
                interval = new double[] {targetX, pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0]};
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
    public double [] findClosestShootPoint(double [] point1, double [] point2, double [] point3, double robotX, double robotY){
        double slope = (point2[0] - point1[0])/(point2[1] - point1[1]);
        double offset = point1[1] - slope * point1[0];
        double min = (robotX + slope * robotY + slope * offset)/(Math.pow(slope,2) + 1);
        if (min < point2[0] && min > point1[0]){
            return new double [] {min, slope*min + offset};
        }
        double p1Dist = Math.pow(Math.pow(robotX - point1[0],2) + Math.pow(robotY - point1[1],2),0.5);
        double p2Dist = Math.pow(Math.pow(robotX - point2[0],2) + Math.pow(robotY - point2[1],2),0.5);
        double [] pointUse;
        if (p1Dist < p2Dist){
            pointUse = point1;
        }else {
            pointUse = point2;
        }
        targetChecked = false;
        //Check which is closer (got rid of the square root for both, because you can do that with inequalities
        if (Math.pow(robotX - pointUse[0],2) + Math.pow(robotY - pointUse[1],2) < Math.pow(robotX - point3[0],2) + Math.pow(robotY - point3[1],2)){
            return new double [] {pointUse[0], pointUse[1]};
        }else{
            return new double [] {point3[0],point3[1]};
        }
    }
    public double[] finalizeTarget(double targetX, double targetY, double heading){
        headingChange = methods.headingChangeRCCXandRCCY(heading);
        double [] newTarget;
        int intersect = checkIntersection(targetX,targetY);
        if (intersect != 16265){
            if (timesAttempted%2 != 0){
                if (targetX - pinpoint.getPosX(DistanceUnit.INCH) - headingChange[0] > targetY - pinpoint.getPosY(DistanceUnit.INCH) - headingChange[1]){
                    newTarget = new double [] {artifactArray[intersect].x,artifactArray[intersect].y + (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1)};
                }else{
                    newTarget = new double [] {artifactArray[intersect].x + (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1), artifactArray[intersect].y};
                }
            }else{
                if (targetX - pinpoint.getPosX(DistanceUnit.INCH) - headingChange[0] > targetY - pinpoint.getPosY(DistanceUnit.INCH) - headingChange[1]){
                    newTarget = new double [] {artifactArray[intersect].x,artifactArray[intersect].y - (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1)};
                }else{
                    newTarget = new double [] {artifactArray[intersect].x - (artifactArray[intersect].radius + robotSize)*(Math.floor(timesAttempted/2)+1), artifactArray[intersect].y};
                }
            }
            targetChecked = false;
            timesAttempted++;
        }else{
            newTarget = new double[] {targetX, targetY};
            targetChecked = true;
            timesAttempted = 0;
        }
        return newTarget;
    }
    public void runOpMode (){
        targetMove = new targetMoveTest(/*hardwareMap*/); //todo fix targetMoveTest to work in another opMode when testing is finished
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        while (opModeIsActive()){
            if (!targetsSet){
                if (reachedArtifact){
                    double [] point1;
                    double [] headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
                    double PosX = pinpoint.getPosX(DistanceUnit.INCH) + headingChange[0];
                    double PosY = pinpoint.getPosY(DistanceUnit.INCH) + headingChange[1];
                    if (isRed){
                        point1 = findClosestShootPoint(new double [] {shotPointsRed[0], shotPointsRed[1]}, new double []{shotPointsRed[2],shotPointsRed[3]}, new double []{shotPointsRed[4],shotPointsRed[5]}, PosX, PosY);
                    }else{
                        point1 = findClosestShootPoint(new double [] {shotPointsBlue[0], shotPointsBlue[1]}, new double []{shotPointsBlue[2],shotPointsBlue[3]}, new double []{shotPointsBlue[4],shotPointsBlue[5]}, PosX, PosY);
                    }
                    targetX = point1[0];
                    targetY = point1[1];
                }else{
                    targetsXY = setArtifactTargets(color);
                    targetX = targetsXY[0];
                    targetY = targetsXY[1];
                }
                targetsSet = true;
            }
            if (targetChecked) {
                targetMove.movementMath();
            } else {
                targetsXY = finalizeTarget(targetX, targetY, pinpoint.getHeading(AngleUnit.RADIANS));
                targetX = targetsXY[0];
                targetY = targetsXY[1];
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY){
                targetsSet = false;
                double slope;
                double offset;
                double [] point;
                if (reachedArtifact){
                    if (isRed){
                        slope = (shotPointsRed[0] - shotPointsRed[2])/(shotPointsRed[1] - shotPointsRed[4]);
                        offset = shotPointsRed[1] - slope * shotPointsRed[0];
                        point = new double [] {shotPointsRed[5],shotPointsRed[6]};
                    }else{
                        slope = (shotPointsBlue[0] - shotPointsBlue[2]) / (shotPointsBlue[1] - shotPointsBlue[4]);
                        offset = shotPointsBlue[1] - slope * shotPointsBlue[0];
                        point = new double [] {shotPointsBlue[5], shotPointsBlue[6]};
                    }
                    if (targetY == slope * targetX + offset || Arrays.equals(new double[]{targetX, targetY}, point)){
                        reachedArtifact = false;
                    }
                }
                for (int j = 0; j < artifactArray.length - 1; j++){
                    if (artifactArray[j].x == targetX && artifactArray[j].y == targetY){
                        artifactArray[j].x = 16265;
                        artifactArray[j].y = 16265;
                        reachedArtifact = true;
                        break;
                    }
                }
            }
            pinpoint.update();
            telemetry.update();
        }
    }
}
