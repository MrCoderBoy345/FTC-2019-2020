package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Simplified autonomous", group = "")
public class AutonomousSimple extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor left4Bar;
    private DcMotor right4Bar;
    private Servo clawServo;

    static final double MAX_POS = .5;
    static  final double MIN_POS = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        left4Bar = hardwareMap.dcMotor.get("left_four_bar");
        right4Bar = hardwareMap.dcMotor.get("right_four_bar");
        clawServo = hardwareMap.servo.get("claw_servo");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        right4Bar.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
            clawServo.setPosition(.5);
            leftDrive.setPower(.5);
            rightDrive.setPower(.5);
            sleep(1200);
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            telemetry.addData("Claw Servo Position", clawServo.getPosition());
            telemetry.addData("Left 4Bar Position", left4Bar.getPower());
            telemetry.addData("Right 4Bar Position", right4Bar.getPower());
            telemetry.addData("Left Power", leftDrive.getPower());
            telemetry.addData("Right Power", rightDrive.getPower());
            telemetry.update();
            sleep(50);
    }
}
