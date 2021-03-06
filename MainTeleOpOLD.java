package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp", group = "")
@Disabled
public class MainTeleOpOLD extends LinearOpMode {

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
        if (opModeIsActive()) {
            clawServo.setPosition(.5);

            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                // we're back to a POV drive now
                // Use left stick to drive and right stick to turn
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                leftDrive.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
                rightDrive.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
                midDrive.setPower(-gamepad1.left_stick_x);

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
                if(gamepad2.a){
                    foundationservo.setPosition(.6);
                }
                if(gamepad2.b){
                    foundationservo.setPosition(0);
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
    }
}
