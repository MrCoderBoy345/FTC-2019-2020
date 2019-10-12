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
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            clawServo.setPosition(.5);

            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Use left stick to drive and right stick to turn
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                // leftDrive.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
                // rightDrive.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);

                // apparently the team would like a tank drive instead of the POV drive
                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                 leftDrive.setPower(gamepad1.left_stick_y);
                 rightDrive.setPower(gamepad1.right_stick_y);

                if (gamepad1.right_bumper){
                    left4Bar.setPower(left4Bar.getPower() + .005); //move 4bar up with the right bumper (kinda finicky right now)
                    right4Bar.setPower(right4Bar.getPower() + .005);
                }
                if (gamepad1.left_bumper){
                    left4Bar.setPower(left4Bar.getPower() - .005); //move 4bar down with the left bumper (kinda finicky right now)
                    right4Bar.setPower(right4Bar.getPower() - .005);
                }

                // to do: change the claw so pressing some button will open it and vice verse for closing

                //if the servo's new position after being changed is greater than what is allowed, DON'T allow it to change in that direction
                if (clawServo.getPosition() + (gamepad1.right_trigger *.03) < MAX_POS && gamepad1.left_trigger == 0) {
                    clawServo.setPosition(clawServo.getPosition() + (gamepad1.right_trigger * .03));
                }
                //^ reversed for LT
                if (clawServo.getPosition() + (gamepad1.left_trigger *-.03) > MIN_POS && gamepad1.right_trigger == 0) {
                    clawServo.setPosition(clawServo.getPosition() + (gamepad1.left_trigger * -.03));
                }

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
