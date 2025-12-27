package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "SimpleAuto", group = "enigma")
public class simpleAuto extends LinearOpMode {
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor intake;
    public DcMotor OuttakeMotor;
    public DcMotor OuttakeMotor2;
    public Servo outtakeServo;
    public double thing = 0;
    public boolean areYouOnBlueTeam = true;
    public enum partsOfAuto {
        move,
        turn,
        shoot,
        intake1,
        shoot2,
        intake2,
        shoot3,
        move2,
        done
    }
    partsOfAuto PartsOfAuto = partsOfAuto.move;
    public double Time;
    public double OutakeServoTime;
    void initialize () {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "RearLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RearRight");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        OuttakeMotor = hardwareMap.get(DcMotor.class, "ShooterRight");
        OuttakeMotor2 = hardwareMap.get(DcMotor.class, "ShooterLeft");
        outtakeServo = hardwareMap.get(Servo.class, "Feeder");

        OuttakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void runOpMode() {
        initialize();
        while (!opModeIsActive()) {
            if (gamepad1.a) {
                areYouOnBlueTeam = false;
            }
        }
        waitForStart();
        sleep(15000);
        Time = System.currentTimeMillis() + 200;
        while (opModeIsActive()) {
            OuttakeMotor.setPower(-0.62);
            OuttakeMotor2.setPower(-0.62);
            if (PartsOfAuto == partsOfAuto.move) {
                if (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightFrontDrive.setPower(1);
                    rightBackDrive.setPower(1);
                }
            }
            if (PartsOfAuto == partsOfAuto.turn && areYouOnBlueTeam) {
                if (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(-1);
                    leftBackDrive.setPower(-1);
                    rightFrontDrive.setPower(1);
                    rightBackDrive.setPower(1);
                }
            }
            if (PartsOfAuto == partsOfAuto.turn && !areYouOnBlueTeam) {
                if (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightFrontDrive.setPower(-1);
                    rightBackDrive.setPower(-1);
                }
            }
            if (PartsOfAuto == partsOfAuto.shoot) {
                if (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    if (OutakeServoTime <= System.currentTimeMillis()) {
                        outtakeServo.setPosition(0.88);
                    }
                }
            }
            if (PartsOfAuto == partsOfAuto.intake1) {
                if (Time > System.currentTimeMillis()) {
                    intake.setPower(0.9);
                }
            }
            if (PartsOfAuto == partsOfAuto.shoot2) {
                if (Time > System.currentTimeMillis()) {
                    intake.setPower(0);
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    if (OutakeServoTime <= System.currentTimeMillis()) {
                        outtakeServo.setPosition(0.91);
                    }
                }
            }
            if (PartsOfAuto == partsOfAuto.intake2) {
                if (Time > System.currentTimeMillis()) {
                    intake.setPower(0.9);
                }
            }
            if (PartsOfAuto == partsOfAuto.shoot3) {
                if (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    if (OutakeServoTime <= System.currentTimeMillis()) {
                        outtakeServo.setPosition(0.91);
                    }
                }
            }
            if (PartsOfAuto == partsOfAuto.move2) {
                while (Time > System.currentTimeMillis()) {
                    leftFrontDrive.setPower(1);
                    leftBackDrive.setPower(1);
                    rightFrontDrive.setPower(1);
                    rightBackDrive.setPower(1);
                }
            }
            if (PartsOfAuto == partsOfAuto.done) {
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                OuttakeMotor.setPower(-0);
                OuttakeMotor2.setPower(-0);
                outtakeServo.setPosition(0.98);
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.move) {
                if (!areYouOnBlueTeam) {
                    Time = System.currentTimeMillis() + 55;
                } else {
                    Time = System.currentTimeMillis() + 45;
                }
                PartsOfAuto = partsOfAuto.turn;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.turn) {
                Time = System.currentTimeMillis() + 3500;
                OutakeServoTime = System.currentTimeMillis() + 3000;
                PartsOfAuto = partsOfAuto.shoot;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.shoot) {
                Time = System.currentTimeMillis() + 500;
                outtakeServo.setPosition(0.98);
                PartsOfAuto = partsOfAuto.intake1;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.intake1) {
                Time = System.currentTimeMillis() + 3500;
                OutakeServoTime = System.currentTimeMillis() + 3000;
                PartsOfAuto = partsOfAuto.shoot2;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.shoot2) {
                Time = System.currentTimeMillis() + 500;
                outtakeServo.setPosition(0.98);
                PartsOfAuto = partsOfAuto.intake2;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.intake2) {
                Time = System.currentTimeMillis() + 3500;
                OutakeServoTime = System.currentTimeMillis() + 3000;
                PartsOfAuto = partsOfAuto.shoot3;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.shoot3) {
                Time = System.currentTimeMillis() + 250;
                PartsOfAuto = partsOfAuto.move2;
            }
            if (!(Time > System.currentTimeMillis()) && PartsOfAuto == partsOfAuto.move2) {
                PartsOfAuto = partsOfAuto.done;
            }
            thing += 1;
            telemetry.addData("thing", thing);
            telemetry.update();

        }
    }
}
