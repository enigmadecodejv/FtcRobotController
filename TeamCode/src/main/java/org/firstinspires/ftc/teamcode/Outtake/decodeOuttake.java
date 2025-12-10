package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class decodeOuttake {
    public Gamepad gamepad1;
    public Servo outtakeServo;
    public DcMotor outtakeMotor;
    public DcMotor outtakeMotor2;
    public Gamepad currentGamepad1;
    public Gamepad pastGamepad1;
    public HardwareMap hardwareMap;
    public boolean shouldTheOutakeMotorsBeOnHighPower = false;

    public decodeOuttake(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        outtakeMotor = hardwareMap.get(DcMotor.class, "ShooterRight");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "ShooterLeft");
        //outtakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo = hardwareMap.get(Servo.class, "Feeder");
        currentGamepad1 = gamepad1;
    }

    public void outtake() {
        pastGamepad1 = currentGamepad1;
        currentGamepad1 = gamepad1;
        if (currentGamepad1.left_trigger > 0.25) {
            outtakeServo.setPosition(0.91);
        } else {
            outtakeServo.setPosition(0.96);
        }
        if (currentGamepad1.a) {
            shouldTheOutakeMotorsBeOnHighPower = false;
        }
        if (currentGamepad1.y) {
            shouldTheOutakeMotorsBeOnHighPower = true;
        }
        if (!shouldTheOutakeMotorsBeOnHighPower) {
            outtakeMotor.setPower(-0.5);
            outtakeMotor2.setPower(-0.5);
        } else {
            outtakeMotor.setPower(-0.7);
            outtakeMotor2.setPower(-0.7);
        }
    }
}
