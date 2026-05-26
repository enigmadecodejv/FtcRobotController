package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DrivetrainTester", group = "Enigma")
public class DrivetrainTester extends LinearOpMode {
    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public double speed = 0.75;
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBack = hardwareMap.get(DcMotor.class, "RearRight");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("Press a for leftFront" +
                    "\nPress b for rightFront" +
                    "\nPress x for rightBack" +
                    "\nPress y for leftBack");
            if (gamepad1.a){
                leftFront.setPower(speed);
            }else{
                leftFront.setPower(0);
            }
            if (gamepad1.b){
                rightFront.setPower(speed);
            }else{
                rightFront.setPower(0);
            }
            if (gamepad1.x){
                rightBack.setPower(speed);
            }else{
                rightBack.setPower(0);
            }
            if (gamepad1.y){
                leftBack.setPower(speed);
            }else{
                leftBack.setPower(0);
            }
            telemetry.update();
        }
    }
}
