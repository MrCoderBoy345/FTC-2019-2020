package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive {
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;

    Drive(DcMotor leftBack, DcMotor rightBack, DcMotor leftFront, DcMotor rightFront){
        this.leftBackDrive = leftBack;
        this.leftFrontDrive = leftFront;
        this.rightBackDrive = rightBack;
        this.rightFrontDrive = rightFront;

        // Reverse motors
        this.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run with encoder
        this.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveLeft(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(-power);
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(power);
    }
    public void moveForward(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }
    public void turnLeft(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
    }
    public void stopMoving(){
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }
}
