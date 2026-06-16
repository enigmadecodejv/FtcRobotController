package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "Enigma")
public class ServoTester extends LinearOpMode {
    public Servo outtakeServo;
    public double servoPos = 0;
    public double lastTime = 0;
    public void runOpMode(){
        outtakeServo = hardwareMap.get(Servo.class, "OuttakeGate");
        waitForStart();
        while(opModeIsActive()){
            if (System.currentTimeMillis() - lastTime > 100){
                servoPos += 0.01;
                lastTime = System.currentTimeMillis();
                outtakeServo.setPosition(servoPos);
            }
            telemetry.addData("ServoPos: ", servoPos);
            telemetry.update();
            if (servoPos >= 3){
                break;
            }
        }
    }
}
