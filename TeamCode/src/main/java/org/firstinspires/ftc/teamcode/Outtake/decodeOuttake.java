package org.firstinspires.ftc.teamcode.Outtake;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class decodeOuttake {
    public Gamepad gamepad1;
    public Servo outtakeGate;
    public DcMotorEx outtakeLeft;
    public DcMotor outtakeRight;
    public Gamepad currentGamepad1;
    public Gamepad pastGamepad1;
    public HardwareMap hardwareMap;
    public boolean shouldTheOutakeMotorsBeOnHighPower = false;
    public boolean lowPower = false;
    public boolean zeroPower = false;

    public decodeOuttake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        outtakeRight = hardwareMap.get(DcMotorEx.class, "OuttakeRight");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "OuttakeLeft");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeGate = hardwareMap.get(Servo.class, "OuttakeGate");
        currentGamepad1 = gamepad1;
        //outtakeLeft.setVelocity(2000);
    }

    public void outtake() {
        /*outtakeRight.setPower(outtakeLeft.getPower());
        if (gamepad1.dpad_down) {
            outtakeLeft.setVelocity(outtakeLeft.getVelocity() - 10);
        } else if (gamepad1.dpad_up) {
            outtakeLeft.setVelocity(outtakeLeft.getVelocity() + 10);
        }*/
        /*if (gamepad1.a) {
            outtakeLeft.setVelocity(2000);
        } else if (gamepad1.y) {
            outtakeLeft.setVelocity(2000);
        }*/
        if (gamepad1.left_trigger > 0.25) {
            outtakeGate.setPosition(0.35);
        } else {
            outtakeGate.setPosition(0.7);
        }
    }
}
