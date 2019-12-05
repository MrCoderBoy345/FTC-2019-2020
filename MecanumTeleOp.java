package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mecanum TeleOp", group = "")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotorSimple leftBackDrive;
    private DcMotorSimple rightBackDrive;
    private DcMotorSimple leftFrontDrive;
    private DcMotorSimple rightFrontDrive;
    private DcMotorSimple left4Bar;
    private DcMotorSimple right4Bar;
    private Servo clawServo;
    private Servo foundationservo1;
    private Servo foundationservo2;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
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

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        right4Bar.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            clawServo.setPosition(.5);

            // Put run blocks here.
            while (opModeIsActive()) {
                float vertical = -gamepad1.left_stick_y;
                float horizontal = gamepad1.left_stick_x;
                float pivot = gamepad1.right_stick_x;

                //check wizards.exe's vid on how to program mecanums
                rightFrontDrive.setPower(vertical - horizontal - pivot);
                rightBackDrive.setPower(vertical + horizontal - pivot);
                leftFrontDrive.setPower(vertical + horizontal + pivot);
                leftBackDrive .setPower(vertical - horizontal + pivot);

                if (gamepad2.left_stick_y < -.03 || gamepad2.left_stick_y > -.03) {
                    left4Bar.setPower(-gamepad2.left_stick_y / 3); //move 4bar down with the left stick of the manipulator controller
                    right4Bar.setPower(-gamepad2.left_stick_y / 3);
                } else{
                    left4Bar.setPower(0);
                    right4Bar.setPower(0);
                }

                //X will close the servo
                if (gamepad2.x) {
                    clawServo.setPosition(1);
                }
                //Y will open the servo
                if (gamepad2.y) {
                    clawServo.setPosition(0.5);
                }
                //open foundation
                if(gamepad2.a){
                    foundationservo1.setPosition(.6);
                }
                //close foundation
                if(gamepad2.b){
                    foundationservo2.setPosition(0);
                }

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
    }
}
