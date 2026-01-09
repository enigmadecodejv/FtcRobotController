package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class followPathAuto extends LinearOpMode {
    DcMotor intakeMotor;
    DcMotor outtakeMotor;
    DcMotor outtakeMotor2;
    Servo outtakeServo;
    public boolean isRed = false;
    public String color = "blue";
    public boolean colorSet = false;
    public double timer = 0;
    public double timer2;
    public boolean timerSet = true;
    public void intake (){
        intakeMotor.setPower(0.8);
    }
    public void outtake (){
        outtakeMotor.setPower(-0.7);
        outtakeMotor2.setPower(0.7);
        if (timer <= System.currentTimeMillis() - 300){
            outtakeServo.setPosition(0.88);
            timerSet = false;
            timer2 = System.currentTimeMillis();
        }else if (!timerSet && timer2 <= System.currentTimeMillis()){
            timer = System.currentTimeMillis();
            outtakeServo.setPosition(0.98);
            timerSet = true;
        }
    }
    public void runOpMode (){
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        while (!opModeIsActive()){
            if (gamepad1.a){
                isRed = true;
                colorSet = true;
                color = "red";
            }
            if (gamepad1.b){
                isRed = false;
                colorSet = true;
                color = "blue";
            }
            if (!colorSet){
                telemetry.addLine("Looking for color. Press a for red, press b for blue");
            }else {
                telemetry.addLine("Team color is " + color + " .");
            }
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){

        }
    }
    public void runRed(){

    }
    public void runBlue(){

    }
}