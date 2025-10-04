package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drive.decodeDriveCode;
import org.firstinspires.ftc.teamcode.Outtake.decodeOuttake;
import org.firstinspires.ftc.teamcode.intake.decodeIntake;

@TeleOp(name="DecodeTeleop", group="Enigma")
public class DecodeTeleop extends LinearOpMode {
    decodeDriveCode driveCode = new decodeDriveCode(gamepad1);
    decodeIntake intakeCode = new decodeIntake();
    decodeOuttake outtakeCode = new decodeOuttake();
    GoBildaPinpointDriver pinpoint;

    double redGoalX = 0, redGoalY = 0;
    double blueGoalX = 0, blueGoalY = 0;
    double goalZ = 0;
    double shootZ = 0;
    double targetVelocity;
    boolean isRed = true;
    boolean shooting;
    boolean shooterIsAtSpeed;
    double gravity = -368.08;
    double timeInAir = 3;
    public double goalDistanceX;
    public double goalDistanceY;
    public double goalDistanceZ = goalZ - shootZ;
    public double goalDistanceXY;
    double quadrant;
    boolean variableAngle = false;

    Pose2D blueGoalPos = new Pose2D(DistanceUnit.INCH, 1.0 ,1.0,AngleUnit.DEGREES,0);
    Pose2D redGoalPos = new Pose2D(DistanceUnit.INCH, 1.0 ,1.0,AngleUnit.DEGREES,0);
    Servo turret;
    DcMotorEx shooter; //dont use Ex for all motors, just use DcMotor
    //Every value in variable shooting code is using seconds and inches
    //Find start velocity and angle for a given distance x, distance y, and time
    public double getShootVelocity(){
        if (isRed){
            goalDistanceX = redGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = redGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }else if (!isRed){
            goalDistanceX = blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }
        goalDistanceXY = Math.pow(Math.pow(goalDistanceX,2) + Math.pow(goalDistanceY,2),0.5);
        double velocityXY = goalDistanceXY/timeInAir;
        double velocityZ = 2*goalDistanceZ + gravity * Math.pow(timeInAir,2)/(2 * timeInAir);
        return Math.pow(Math.pow(velocityXY,2) + Math.pow(velocityZ,2),0.5);
    }
    public double getShootAngle(){
        if (isRed){
            goalDistanceX = redGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = redGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }else if (!isRed){
            goalDistanceX = blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }
        goalDistanceXY = Math.pow(Math.pow(goalDistanceX,2) + Math.pow(goalDistanceY,2),0.5);
        double velocityXY = goalDistanceXY/timeInAir;
        double velocityZ = 2*goalDistanceZ + gravity * Math.pow(timeInAir,2)/(2 * timeInAir);
        double angleRad = Math.atan(velocityZ / velocityXY);
        if (velocityXY > 0 && velocityZ > 0){
            quadrant = 1;
        }else if (velocityXY < 0 && velocityZ > 0){
            quadrant = 2;
        }else if (velocityXY < 0 && velocityZ < 0){
            quadrant = 3;
        }else if (velocityXY > 0 && velocityZ < 0){
            quadrant = 4;
        }
        if (angleRad > Math.PI*quadrant/2 || angleRad < Math.PI*(quadrant-1)/2){
            angleRad += Math.PI;
        }
        if (angleRad < 0){
            angleRad += 2*Math.PI;
        }else if (angleRad > 2*Math.PI){
            angleRad -= 2*Math.PI;
        }
        return angleRad;
    }
    public void variableOuttake(double speed, double angle){
        /*
         I don't know how to make the motor release the balls at a certain speed, so will need to figure that out
         Pretty sure how that works is that launch velocity = total velocity - backspin and backspin is proportional to total velocity
        I think total velocity = rpm(motor) * circumference of motor or thing the motor is attached to
        Use speed for what the launch velocity should be
        Experiment to find the multiplier of total velocity to get backspin
        */
        turret.setPosition(angle/(2*Math.PI));//Angle the turret servo properly
    }

    //For a given angle, x, and y distance, find start velocity
    double angle = 0;
    public double getVelocityShot(){
        if (isRed){
            goalDistanceX = redGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = redGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }else if (!isRed){
            goalDistanceX = blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }
        goalDistanceXY = Math.pow(Math.pow(goalDistanceX,2) + Math.pow(goalDistanceY,2),0.5);
        double velocityXY = goalDistanceXY * Math.pow(gravity,0.5)/Math.pow(2 * (goalDistanceZ - goalDistanceXY * Math.tan(angle * Math.PI / 180)),0.5);
        double velocityZ = velocityXY * Math.tan(angle*Math.PI/180);
        return Math.pow(Math.pow(velocityXY,2)+Math.pow(velocityZ,2),0.5);
    }
    public void autoOuttake(double speed){
        /*
         I don't know how to make the motor release the balls at a certain speed, so will need to figure that out
         Pretty sure how that works is that launch velocity = total velocity - backspin and backspin is proportional to total velocity
        I think total velocity = rpm(motor) * circumference of motor or thing the motor is attached to
        Use speed for what the launch velocity should be
        Experiment to find the multiplier of total velocity to get backspin
        */
    }
    /*public double getDistanceToGoal() {
        double robotPosX, robotPosY;

        robotPosX = pinpoint.getPosX(DistanceUnit.INCH);
        robotPosY = pinpoint.getPosY(DistanceUnit.INCH);

        Pose2D goal = redGoalPos;

        if (!isRed) {
            goal = blueGoalPos; // You can expand this later for blue goal if needed
        }

        double dx = robotPosX - goal.getX(DistanceUnit.INCH);
        double dy = robotPosY - goal.getY(DistanceUnit.INCH);

        return Math.sqrt(dx * dx + dy * dy);
    }

    

    public double angleToServoPos(double angle) {
        double denominator = 360;
        if(angle == 0) {
            angle = 0.001;
        }

        return angle/denominator;
    }

    public static double getAngleBetweenPoints(double x1, double y1, double x2, double y2) {
        // diff in coords (delta)
        double deltaY = y2 - y1;
        double deltaX = x2 - x1;

        // atan2 to get angle in radians, according to wikipedia
        double angleInRadians = Math.atan2(deltaY, deltaX);

        double angleInDegrees = Math.toDegrees(angleInRadians);

        if (angleInDegrees < 0) {
            angleInDegrees += 360;
        }

        return angleInDegrees;
    }

    public void shootLogic() {
        while(shooting) {
            aimAtGoal();
            setShooterSpeedBasedOffDistance();
            if (!shooterIsAtSpeed) {
                shooter.setVelocity(targetVelocity);
            }
        }


        //todo make this work
    }

    public void shoot() {
        shooting = true;
    }

    public void setShooterSpeedBasedOffDistance() {
        final double denominatorConstant = 67;

        targetVelocity = getDistanceToGoal()/denominatorConstant; //not sure how this will exactly work but this is the idea
    }
    public double rpmToTPR() {
        return 1;
    }

    public boolean shooterIsAtSpeed() {
        double tpr = 28; //todo possibly change this depending on which motor we're using, 28 is based off the gobilda 6000rpm motor
        double velocityTicksPerSecond = shooter.getVelocity();
        double rpm = (velocityTicksPerSecond / tpr) * 60;

        if(rpm > targetVelocity) {
            return true;
        }
        else {
            return false;
        }
    }

    public void aimAtGoal() {
        double rPosX = pinpoint.getPosX(DistanceUnit.INCH);
        double rPosY = pinpoint.getPosY(DistanceUnit.INCH);
        if(isRed) {
            SmartServo.setSmartPos(hardwareMap, "turret", angleToServoPos(getAngleBetweenPoints(rPosX, rPosY, redGoalPos.getX(DistanceUnit.INCH), redGoalPos.getY(DistanceUnit.INCH))));
        }
        if(!isRed) {
            SmartServo.setSmartPos(hardwareMap, "turret", angleToServoPos(getAngleBetweenPoints(rPosX, rPosY, blueGoalPos.getX(DistanceUnit.INCH), blueGoalPos.getY(DistanceUnit.INCH))));
        }
    }
*/
    private void mainLoop() {
        //run functions
        driveCode.runWheels();
        intakeCode.intake(hardwareMap);
        outtakeCode.outtake();
        if (variableAngle) {
            variableOuttake(getShootVelocity(), getShootAngle());
        }else if (!variableAngle){
            autoOuttake(getVelocityShot());
        }
    }
    void initialize() {
        //hardware
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = hardwareMap.get(Servo.class,"turret");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            mainLoop();
        }
    }

}
