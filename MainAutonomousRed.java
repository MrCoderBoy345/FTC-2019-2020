package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Main Red Auto", group = "")
@Disabled
public class MainAutonomousRed extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotorSimple left4Bar;
    private DcMotorSimple right4Bar;
    private Servo clawServo;
    private Servo foundationservo1;
    private Servo foundationservo2;
    private ColorSensor skystoneDetector;
    private DistanceSensor frontDistance;

    /**
     * move left, backup until it sees a skystone, pick it up, go forward, drop in the build site, backup,ove
     */
    @Override
    public void runOpMode() {
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        left4Bar = (DcMotorSimple) hardwareMap.get("left_four_bar");
        right4Bar = (DcMotorSimple) hardwareMap.get("right_four_bar");
        clawServo = hardwareMap.servo.get("claw_servo");
        foundationservo1 = hardwareMap.servo.get("foundation_servo_1");
        foundationservo2 = hardwareMap.servo.get("foundation_servo_2");
        skystoneDetector = hardwareMap.colorSensor.get("skystone_color_sensor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "skystone_color_sensor");


        // Reverse some of the motors.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        left4Bar.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            clawServo.setPosition(.4);
            // foundationServo.setPosition();

            // Run with encoder
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            moveForward(.5);
            sleep(1300);
            stopMoving();
            moveLeft(.75);
            while ((skystoneDetector.red() / skystoneDetector.blue()) * (skystoneDetector.green() / skystoneDetector.blue()) > 0.2){
                //wait for it (being used as a sleep function)
            }
            moveLeft(-.75);
            while ((skystoneDetector.red() / skystoneDetector.blue()) * (skystoneDetector.green() / skystoneDetector.blue()) < 0.2){
                //wait for it (being used as a sleep function)
            }
            stopMoving();
            moveForward(.5);
            sleep(500);
            stopMoving();
            clawServo.setPosition(1); //grab the stone
            sleep(500);
            moveForward(-.75);
            sleep(500);
            stopMoving();
            turnLeft(-1);
            sleep(600);
            stopMoving();
            moveLeft(.5);
            sleep(750);
            stopMoving();
            sleep(500);
            moveForward(1); //drive to build site
            sleep(1000);
            stopMoving();
            sleep(1000);
            turnLeft(1);
            sleep(1000);
            stopMoving();
            clawServo.setPosition(.6); //release the skystone
            sleep(2000);
            turnLeft(-1); // line up with the foundation
            sleep(1500);
            stopMoving();
            moveForward(-.5);
            sleep(1200);
            stopMoving();
            moveLeft(.5);
            sleep(750);
            stopMoving();
            // repeat the foundation code from FoundationRed.java
            foundationservo1.setPosition(1);  //grab the foundation
            foundationservo2.setPosition(1);
            sleep(2000);
            moveForward(1);
            sleep(2500);
            stopMoving();
            turnLeft(-1);
            sleep(1000);
            stopMoving();
            sleep(500);
            moveForward(-1);
            sleep(3000);
            stopMoving();
            foundationservo1.setPosition(0);
            foundationservo2.setPosition(0);
            sleep(1000);
            moveForward(.75);
            sleep(1200);
            moveLeft(-1);
            sleep(1300);
            stopMoving();
            moveForward(.75);
            sleep(1000);
            stopMoving();

            telemetry.addData("Claw Servo Position", clawServo.getPosition());
            telemetry.addData("Left 4Bar Power", left4Bar.getPower());
            telemetry.addData("Right 4Bar Power", right4Bar.getPower());
            telemetry.addData("Left Back Power", leftBackDrive.getPower());
            telemetry.addData("Right Back Power", rightBackDrive.getPower());
            telemetry.addData("Left Front Power", leftFrontDrive.getPower());
            telemetry.addData("Right Front Power", rightFrontDrive.getPower());
            telemetry.update();
            sleep(50);
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
