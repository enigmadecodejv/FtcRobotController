package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class decodeOuttake {
    public Gamepad gamepad2;
    public Servo outtakeServo;
    public DcMotor outtakeMotor;
    public Gamepad currentGamepad2;
    public Gamepad pastGamepad2;
    public void outtake (){
        pastGamepad2 = currentGamepad2;
        currentGamepad2 = gamepad2;
        if (currentGamepad2.left_trigger > 0.1 && pastGamepad2.left_trigger < 0.1) {
            outtakeServo.setPosition(outtakeServo.getPosition() + 1);
        }
        if (gamepad2.left_trigger > 0.1){
            outtakeMotor.setPower(gamepad2.left_trigger);
        }
    }
}
