package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class decodeDriveCode {
    //define gamepad1
    public Gamepad gamepad1;
    public double drive;

    // declare wheels
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public double speed = 0.75;

    public decodeDriveCode(Gamepad gamepad1, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        this.gamepad1 = gamepad1;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void runWheels() {
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.\
        double boostMultiplier = 1.2;

        drive = -gamepad1.left_stick_y * boostMultiplier;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x * boostMultiplier;


        // Combine drive and turn for blended motion.
        // variables
        double leftFront = drive + turn + strafe;
        double leftBack = drive + turn - strafe;
        double rightFront = drive - turn - strafe;
        double rightBack = drive - turn + strafe;

        // Normalize the values so neither exceed +/- 1.0

        double max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        if (max > 1.0)
        {
            leftFront /= max;
            rightFront /= max;
        }



        // Output the safe vales to the motor drives.
        leftFrontDrive.setPower(leftFront * speed);
        leftBackDrive.setPower(leftBack * speed);
        rightFrontDrive.setPower(rightFront * speed);
        rightBackDrive.setPower(rightBack * speed);
    }

    public boolean areWheelsMoving(){
        if (leftFrontDrive.getPower() == 0 && rightFrontDrive.getPower() == 0){
            return false;
        }else {
            return true;
        }
    }
}
