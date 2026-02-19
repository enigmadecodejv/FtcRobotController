package org.firstinspires.ftc.teamcode.Outtake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class PIOuttake {
    public Gamepad gamepad1;
    public Servo outtakeServo;
    public DcMotorEx outtakeMotor;
    public DcMotor outtakeMotor2;
    public Gamepad currentGamepad1;
    public Gamepad pastGamepad1;
    public HardwareMap hardwareMap;
    public boolean shouldTheOutakeMotorsBeOnHighPower = false;
    public boolean lowPower = false;
    public boolean zeroPower = false;
    public double integral = 0;
    public double derivative;
    public double motorSpeed;
    public double error;
    public double timer;
    public double lastError = 33778;
    public static double kp = 0.01;
    public static double ti = 120;
    public static double td = 0;
    public double integral2 = 0;
    public static double farSpeed = 1425;
    public static double closeSpeed = 1150;
    public double speedPID = closeSpeed;

    public PIOuttake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo = hardwareMap.get(Servo.class, "OuttakeGate");
        currentGamepad1 = gamepad1;
    }
    public double outtakePID(double targetSpeed){
        error = targetSpeed - outtakeMotor.getVelocity();
        integral += error;
        integral2 += error;
        timer += 1;
        if (lastError != 33778) {
            derivative = (error - lastError) / timer;
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
        }else if (currentGamepad1.y){
            speedPID = farSpeed;
        }else if (currentGamepad1.dpad_down || currentGamepad1.dpad_left || currentGamepad1.dpad_up || currentGamepad1.dpad_right){
            speedPID = 0;
        }
        double targetPower = outtakePID(speedPID);
        if(targetPower > 1){
            targetPower = 1;
        }else if (targetPower < -1){
            targetPower = -1;
        }
        outtakeMotor.setPower(targetPower);
        outtakeMotor2.setPower(targetPower);
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeServo.setPosition(0.35);
        } else {
            outtakeServo.setPosition(0.7);
        }
    }
    public void outtake() {
        pastGamepad1 = currentGamepad1;
        currentGamepad1 = gamepad1;
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeServo.setPosition(0.35);
        } else {
            outtakeServo.setPosition(0.7);
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
    }
}