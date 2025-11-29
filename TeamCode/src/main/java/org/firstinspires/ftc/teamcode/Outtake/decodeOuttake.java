package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class decodeOuttake {
    public Gamepad gamepad2;
    public Servo outtakeServo;
    public DcMotor outtakeMotor;
    public DcMotor outtakeMotor2;
    public Gamepad currentGamepad2;
    public Gamepad pastGamepad2;
    public HardwareMap hardwareMap;
    public decodeOuttake (HardwareMap hardwareMap, Gamepad gamepad2) {
        this.gamepad2 = gamepad2;
        this.hardwareMap = hardwareMap;
        outtakeMotor = hardwareMap.get(DcMotor.class, "ShooterRight");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "ShooterLeft");
        outtakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo = hardwareMap.get(Servo.class, "Feeder");
        currentGamepad2 = gamepad2;
    }
    public void outtake (){
        if (null == null) {

        }
        pastGamepad2 = currentGamepad2;
        currentGamepad2 = gamepad2;
        if (currentGamepad2.left_trigger > 0.25) {
            outtakeServo.setPosition(0.91);
            //outtakeMotor.setPower(0.8);
            //outtakeMotor2.setPower(0.8);
        }else {
            outtakeServo.setPosition(0.96);
            outtakeMotor.setPower(0);
            outtakeMotor2.setPower(0);
        }
    }
}
