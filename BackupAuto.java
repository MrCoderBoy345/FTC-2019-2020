package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Backup Autonomous", group = "")

public class BackupAuto extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private Servo clawServo;
    private Servo foundationservo1;
    private Servo foundationservo2;

    /**
     * move left, backup until it sees a skystone, pick it up, go forward, drop in the build site, backup,
     */
    @Override
    public void runOpMode() {
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        clawServo = hardwareMap.servo.get("claw_servo");
        foundationservo1 = hardwareMap.servo.get("foundation_servo_1");
        foundationservo2 = hardwareMap.servo.get("foundation_servo_2");

        // Reverse some of the motors.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            // Run with encoder
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(15000);
            moveForward(1);
            sleep(1400);
            stopMoving();
            clawServo.setPosition(.4);
            sleep(3000);
        }
    }
    private void moveLeft(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
    }
    private void moveForward(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }
    private void turnLeft(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
    }
    private void stopMoving(){
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

}
