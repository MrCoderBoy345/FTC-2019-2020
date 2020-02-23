package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "encoder test", group = "")
public class EncoderTest extends LinearOpMode {

    private static final float mmPerInch = 25.4f;
    private static final int ticksPerRevolution = 288;
    private static final int mmPerRevolution = 90;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorSimple midDrive;
    private DcMotor left4Bar;
    private DcMotor right4Bar;
    private Servo clawServo;
    private Servo foundationservo;

    /**
     * If this works, the robot will travel approximately 12 inches forward
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
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            midDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            clawServo.setPosition(.4);
            foundationservo.setPosition(.6);

            DriveForward(1, 12);
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

    /**
     * allow us to drive forward using encoders
     * to reverse, put in a negative power
     * distance is in inches
     */
    public void DriveForward(double power, int distance){
        /**
         * inches * mmPerIn for mm
         * divided by distancePerRevolution for total revolutions
         * times ticksPerRevolution for total ticks
         */
        int ticks = (int) (distance * mmPerInch)/mmPerRevolution*ticksPerRevolution;

        if (opModeIsActive()) {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setTargetPosition(ticks);
            rightDrive.setTargetPosition(ticks);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(power);
            rightDrive.setPower(power);

            while(leftDrive.isBusy() || rightDrive.isBusy()){
                //wait for it
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
