package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp", group = "")
public class MainTeleOp extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor left4Bar;
    private DcMotor right4Bar;
    private Servo clawServo;

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
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                // Use left stick to drive and right stick to turn
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                leftDrive.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
                rightDrive.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);

                left4Bar.setPower(left4Bar.getPower() + gamepad1.left_stick_y/8); //attempt to move 4bar up/down with the joystick (NOT WORKING YET)
                right4Bar.setPower(right4Bar.getPower() + gamepad1.right_stick_y/8);

                clawServo.setPosition(clawServo.getPosition() + (gamepad1.right_stick_x *.01));

                telemetry.addData("Claw Servo Position", clawServo.getPosition());
                telemetry.addData("Left 4Bar Position", left4Bar.getPower());
                telemetry.addData("Right 4Bar Position", right4Bar.getPower());
                telemetry.addData("Left Power", leftDrive.getPower());
                telemetry.addData("Right Power", rightDrive.getPower());
                telemetry.update();
                sleep(50);
            }
        }
    }
}
