package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class decodeIntake {
    public Gamepad gamepad1;
    public DcMotor intakeRight, intakeLeft;
    public HardwareMap hardwareMap;
    public double intakePower;
    public boolean movingForward;
    public decodeIntake (HardwareMap hardwareMap, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.hardwareMap = hardwareMap;
        intakeRight = hardwareMap.get(DcMotor.class, "IntakeRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "IntakeLeft");
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    public void intake (){
        if (movingForward){
            intakePower = 1;
        }else if (!movingForward){
            intakePower = 1;
        }
        if (gamepad1.right_trigger > 0.25){
            intakeRight.setPower(intakePower);
            intakeLeft.setPower(intakePower);
        }else if (gamepad1.right_bumper){
            intakeRight.setPower((-0.5) * intakePower);
            intakeLeft.setPower((-0.5) * intakePower);
        }else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
    }
}
