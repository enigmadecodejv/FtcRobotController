package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drive.decodeDriveCode;
import org.firstinspires.ftc.teamcode.Outtake.decodeOuttake;
import org.firstinspires.ftc.teamcode.intake.decodeIntake;
@Configurable
@TeleOp(name="DecodeTeleop", group="Enigma")
public class DecodeTeleop extends LinearOpMode {
    public GoBildaPinpointDriver pinpoint;
    decodeDriveCode driveCode;
    decodeIntake intakeCode;
    decodeOuttake outtakeCode;
    DcMotorEx outtakeMotor;
    DcMotor outtakeMotor2;
    Servo outtakeServo;

    double redGoalX = 130, redGoalY = 138.25;
    double blueGoalX = 11, blueGoalY = 138.25;
    double goalZ = 46.25;
    double shootZ = 10.5;
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
    double motorRotSpeed = 6000 * 2 * Math.PI/60;
    double motorRadius = 4.25/2.54;
    boolean variableAngle = false;
    public double outtakeVelocity;
    public double outputPID;
    private TelemetryManager manager;

    enum PowerGood{
        yes,
        tooClose,
        tooFar
    }
    public PowerGood powerGood;
    public double power;

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
        if (angleRad < 0){
            angleRad += 2*Math.PI;
        }else if (angleRad > 2*Math.PI){
            angleRad -= 2*Math.PI;
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
    double angle = Math.PI/4;
    /*public double getVelocityShot(){
        if (isRed){
            goalDistanceX = redGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = redGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }else if (!isRed){
            goalDistanceX = blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }
        goalDistanceXY = Math.pow(Math.pow(goalDistanceX,2) + Math.pow(goalDistanceY,2),0.5);
        if (goalDistanceZ - Math.tan(angle) * goalDistanceXY == 0){
            return Math.pow(-1,0.5);
        }
        double velocityXY = Math.pow(gravity*Math.pow(goalDistanceXY,2)/(2*(goalDistanceZ - Math.tan(angle) * goalDistanceXY)),0.5);
        double velocityZ = velocityXY * Math.tan(angle);
        return Math.pow(Math.pow(velocityXY,2)+Math.pow(velocityZ,2),0.5);
    }*/
    public double getVelocityShot(){
        if (isRed){
            goalDistanceX = redGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = redGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }else if (!isRed){
            goalDistanceX = blueGoalX - pinpoint.getPosX(DistanceUnit.INCH);
            goalDistanceY = blueGoalY - pinpoint.getPosY(DistanceUnit.INCH);
        }
        return Math.sqrt(Math.pow(goalDistanceX,2) * gravity/(goalDistanceX * Math.sin(2 * angle) - 2 * goalDistanceY * Math.pow(Math.cos(angle),2)));
    }
    public void autoOuttake(double speed){
        powerGood = PowerGood.yes;
        if (speed != speed){
            powerGood = PowerGood.tooClose;
            power = 0;
        }else {
            power = speed/ (motorRotSpeed * motorRadius);
            if (power > 0.8) {
                power = 0.8;
                powerGood = PowerGood.tooFar;
            } else if (power < -0.8) {
                power = -0.8;
                powerGood = PowerGood.tooFar;
            }
        }
        outtakeMotor.setPower(power);
        outtakeMotor2.setPower(power);
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
        /*if (gamepad2.left_trigger > 0.25 && powerGood == PowerGood.yes){
            outtakeServo.setPosition(0.91);
            telemetry.addLine("Shooting");
        }else if (powerGood == PowerGood.tooFar){
            telemetry.addLine("Get closer to the goal.");
        }else if (powerGood == PowerGood.tooClose){
            telemetry.addLine("Get farther from the goal");
        }
        if (gamepad2.left_trigger <= 0.25) {
            outtakeServo.setPosition(0.96);
        }
        telemetry.addData("power: ",power);
        telemetry.addData("speed:", getVelocityShot());
        autoOuttake(getVelocityShot());*/
        driveCode.runWheels();
        intakeCode.intake();
        outtakeCode.runUsingPID();
        outtakeVelocity = outtakeMotor.getVelocity();
        telemetry.addData("Motor Velocity: ", outtakeVelocity);
        manager.addData("MotorVelocity", outtakeVelocity);
        manager.addData("Integral", outtakeCode.integral);
        manager.addData("Integral2", outtakeCode.integral2);
        manager.addData("error", outtakeCode.error);
        if (decodeOuttake.ti == 0){
            manager.addData("Integral/ti", 0);
        }else {
            manager.addData("Integral/ti", outtakeCode.integral/decodeOuttake.ti);
        }
        manager.update();
        telemetry.update();
        pinpoint.update();
    }
    void initialize() {
        //hardware
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        manager = PanelsTelemetry.INSTANCE.getTelemetry();
        driveCode = new decodeDriveCode(gamepad1, hardwareMap);
        intakeCode = new decodeIntake(hardwareMap, gamepad1);
        outtakeCode = new decodeOuttake(hardwareMap, gamepad1);
        if (driveCode.drive > 0.25){
            intakeCode.movingForward = true;
        }else {
            intakeCode.movingForward = false;
        }
    }

    @Override
    public void runOpMode() {
        initialize();
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "ShooterLeft");
        outtakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo = hardwareMap.get(Servo.class, "Feeder");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"PinPoint");
        pinpoint.setOffsets(2, 2.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.resetPosAndIMU();
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        waitForStart();


        while (opModeIsActive()) {
            mainLoop();
        }
    }

}
