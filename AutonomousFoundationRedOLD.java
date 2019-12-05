package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous Foundation Movement For If Red", group = "")
@Disabled
public class AutonomousFoundationRedOLD extends LinearOpMode {

    private static final float mmPerInch = 25.4f;
    private static final int ticksPerRevolution = 288;
    private static final int mmPerRevolution = 90;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor midDrive;
    private DcMotor left4Bar;
    private DcMotor right4Bar;
    private Servo clawServo;
    private Servo foundationservo;

    /**
     * This auto is supposed to move the foundation during autonomous
     */
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        midDrive = hardwareMap.dcMotor.get("middle_drive");
        left4Bar = (DcMotor) hardwareMap.get("left_four_bar");
        right4Bar = (DcMotor) hardwareMap.get("right_four_bar");
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
            midDrive.setPower(.5);
            sleep(1040);
            midDrive.setPower(0);
            leftDrive.setPower(-.5);
            rightDrive.setPower(-.5);
            sleep(2000);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            foundationservo.setPosition(0);
            sleep(1000);
            leftDrive.setPower(1);
            rightDrive.setPower(.97);
            sleep(2500);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            foundationservo.setPosition(.6);
            sleep(1000);
            rightDrive.setPower(-.75);
            //leftDrive.setPower(-.75);
            sleep(600);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            midDrive.setPower(-1);
            sleep(3000);
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
