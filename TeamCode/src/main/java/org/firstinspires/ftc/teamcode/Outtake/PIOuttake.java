package org.firstinspires.ftc.teamcode.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class PIOuttake {
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public Servo outtakeServo0;
    public Servo outtakeServo1;
    public DcMotorEx outtakeMotor;
    public DcMotor outtakeMotor2;
    public Gamepad currentGamepad1;
    public Gamepad currentGamepad2;
    public Gamepad pastGamepad1;
    public HardwareMap hardwareMap;
    public boolean shouldTheOutakeMotorsBeOnHighPower = false;
    public boolean lowPower = false;
    public boolean zeroPower = false;
    public double integral = 0;
    public double derivative;
    public double error;
    public double timer = 0;
    public double lastError = 33778;
    public static double kp = 0.015;
    public static double ti = 60;
    public static double td = 0;
    public double integral2 = 0;
    public static double farSpeed = 1650; // for now, real speed is 1425
    public static double closeSpeed = 1150;
    public double speedPID = closeSpeed;
    public Servo LED;
    public static double derivativeThreshhold = 0;
    public static double outtakeServo0OpenPosition = 0.7;
    public static double outtakeServo0ClosedPosition = 0.44;
    public static double outtakeServo1OpenPosition = 1.0;
    public static double outtakeServo1ClosedPosition = 0.75;

    public PIOuttake(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeServo0 = hardwareMap.get(Servo.class, "OuttakeGateRight");
        outtakeServo1 = hardwareMap.get(Servo.class, "OuttakeGateLeft");
        LED = hardwareMap.get(Servo.class, "RGBLightIndicator");

        currentGamepad1 = gamepad1;
        currentGamepad2 = gamepad2;
    }
    public double outtakePID(double targetSpeed){
        error = targetSpeed - outtakeMotor.getVelocity();
        integral += error;
        integral2 += error;
        if (lastError != 33778) {
            derivative = (error - lastError) / (System.currentTimeMillis() - timer);
        }
        timer = System.currentTimeMillis();
        if (derivative * td <= derivativeThreshhold) {
            derivative = 0;
        }
        lastError = error;
        if (ti == 0){
            return kp*(error + td*derivative);
        }
        return kp*(error + integral/ti + td*derivative);
    }
    public void runUsingPID(){
        if (currentGamepad1.a){
            speedPID = closeSpeed;
            LED.setPosition(0.621);
        }else if (currentGamepad1.y){
            speedPID = farSpeed;
            LED.setPosition(0.287);
        }else if (currentGamepad1.dpad_down || currentGamepad1.dpad_left || currentGamepad1.dpad_up || currentGamepad1.dpad_right){
            speedPID = 0;
            LED.setPosition(0.510);
        }
        double targetPower = outtakePID(speedPID);
        if(targetPower > 0.8){
            targetPower = 0.8;
        }else if (targetPower < -0.8){
            targetPower = -0.8;
        }
        outtakeMotor.setPower(targetPower);
        outtakeMotor2.setPower(targetPower);
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeServo0.setPosition(outtakeServo0OpenPosition);
            outtakeServo1.setPosition(outtakeServo1OpenPosition);
        } else {
            outtakeServo0.setPosition(outtakeServo0ClosedPosition);
            outtakeServo1.setPosition(outtakeServo1ClosedPosition);
        }
    }
    /*public void outtake() {
        pastGamepad1 = currentGamepad1;
        currentGamepad1 = gamepad1;
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeServo.setPosition(0.35);
        } else {
            outtakeServo.setPosition(0.5);
        }
        if (currentGamepad1.a) {
            shouldTheOutakeMotorsBeOnHighPower = false;
            zeroPower = false;
        }
        if (currentGamepad1.y) {
            shouldTheOutakeMotorsBeOnHighPower = true;
            zeroPower = false;
        }
        if (currentGamepad1.b || currentGamepad1.x) {
            lowPower = true;
            zeroPower = false;
        }
        if (currentGamepad1.dpad_down || currentGamepad1.dpad_left || currentGamepad1.dpad_up || currentGamepad1.dpad_right){
            zeroPower = true;
        }
        if (zeroPower){
            outtakeMotor.setPower(0);
            outtakeMotor2.setPower(0);
        }else if (!shouldTheOutakeMotorsBeOnHighPower && lowPower) {
            outtakeMotor.setPower(-0.55);
            outtakeMotor2.setPower(-0.55);
        } else if (!shouldTheOutakeMotorsBeOnHighPower) {
            outtakeMotor.setPower(-0.6);
            outtakeMotor2.setPower(-0.6);
        } else {
            outtakeMotor.setPower(-0.65);
            outtakeMotor2.setPower(-0.65);
        }
    }*/
}