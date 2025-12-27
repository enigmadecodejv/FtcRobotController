package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.PurpleOrGreen;

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
    int timesAttempted = 1;
    double robotSize = 12;
    public double [] headingChange;
    public double [] targetsXY;
    public boolean targetsSet = false;
    public void setArtifactTargets(PurpleOrGreen color){
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
        targetX = artifactArray[artifactArrayPos].x;
        targetY = artifactArray[artifactArrayPos].y;
        targetChecked = false;
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
    public double[] finalizeTarget(double targetX, double targetY){
        headingChange = methods.headingChangeRCCXandRCCY(pinpoint.getHeading(AngleUnit.RADIANS));
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
        }else{
            newTarget = new double[] {targetX, targetY};
            targetChecked = true;
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
                setArtifactTargets(color);
                targetsSet = true;
            }
            if (targetChecked) {
                targetMove.movementMath();
            } else {
                targetsXY = finalizeTarget(targetX, targetY);
                targetX = targetsXY[0];
                targetY = targetsXY[1];
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
            }
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY){
                targetsSet = false;
            }
            if (pinpoint.getPosX(DistanceUnit.INCH) == targetX && pinpoint.getPosY(DistanceUnit.INCH) == targetY){
                for (int j = 0; j < artifactArray.length - 1; j++){
                    if (artifactArray[j].x == targetX && artifactArray[j].y == targetY){
                        artifactArray[j].x = 16265;
                        artifactArray[j].y = 16265;
                        break;
                    }
                }
            }
            pinpoint.update();
            telemetry.update();
        }
    }
}
