package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Foundation Movement For If Blue", group = "")
public class AutonomousFoundationBlue extends LinearOpMode {

    private DcMotorSimple leftDrive;
    private DcMotorSimple rightDrive;
    private DcMotorSimple midDrive;
    private DcMotorSimple left4Bar;
    private DcMotorSimple right4Bar;
    private Servo clawServo;
    private Servo foundationservo;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        midDrive = (DcMotorSimple) hardwareMap.get("middle_drive");
        left4Bar = hardwareMap.dcMotor.get("left_four_bar");
        right4Bar = hardwareMap.dcMotor.get("right_four_bar");
        clawServo = hardwareMap.servo.get("claw_servo");
        foundationservo = hardwareMap.servo.get("foundation_servo");


        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        right4Bar.setDirection(DcMotorSimple.Direction.REVERSE);
        midDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()){
            clawServo.setPosition(.4);
            foundationservo.setPosition(.6);
            midDrive.setPower(-.5);
            sleep(1500);
            midDrive.setPower(0);
            leftDrive.setPower(-.5);
            rightDrive.setPower(-.5);
            sleep(2000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            foundationservo.setPosition(0);
            sleep(1000);
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            sleep(2000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            foundationservo.setPosition(.6);
            sleep(1000);
            midDrive.setPower(1);
            sleep(2600);
            midDrive.setPower(0);
        }

        telemetry.addData("Claw Servo Position", clawServo.getPosition());
        telemetry.addData("Left 4Bar Power", left4Bar.getPower());
        telemetry.addData("Right 4Bar Power", right4Bar.getPower());
        telemetry.addData("Middle Power", midDrive.getPower());
        telemetry.addData("Left Power", leftDrive.getPower());
        telemetry.addData("Right Power", rightDrive.getPower());
        telemetry.update();
        sleep(50);
    }
}
