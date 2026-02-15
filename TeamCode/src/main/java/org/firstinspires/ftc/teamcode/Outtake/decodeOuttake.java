package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class decodeOuttake {
    public Gamepad gamepad1;
    public Servo outtakeGate;
    public DcMotor outtakeRight, outtakeLeft;
    public Gamepad currentGamepad1;
    public Gamepad pastGamepad1;
    public HardwareMap hardwareMap;
    public boolean shouldTheOutakeMotorsBeOnHighPower = false;
    public boolean lowPower = false;
    public boolean zeroPower = false;

    public decodeOuttake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        outtakeRight = hardwareMap.get(DcMotor.class, "OuttakeRight");
        outtakeLeft = hardwareMap.get(DcMotor.class, "OuttakeLeft");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeGate = hardwareMap.get(Servo.class, "OuttakeGate");
        currentGamepad1 = gamepad1;
    }

    public void outtake() {
        pastGamepad1 = currentGamepad1;
        currentGamepad1 = gamepad1;
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeGate.setPosition(0.45);
        } else {
            outtakeGate.setPosition(0.05);
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
        outtakeRight.setPower(0);
        outtakeLeft.setPower(0);
//        if (zeroPower){
//            outtakeRight.setPower(0);
//            outtakeLeft.setPower(0);
//        }else if (!shouldTheOutakeMotorsBeOnHighPower && lowPower) {
//            outtakeRight.setPower(-0.6);
//            outtakeLeft.setPower(-0.6);
//        } else if (!shouldTheOutakeMotorsBeOnHighPower) {
//            outtakeRight.setPower(-0.6);
//            outtakeLeft.setPower(-0.6);
//        } else {
//            outtakeRight.setPower(-0.75);
//            outtakeLeft.setPower(-0.75);
//        }
    }
}
