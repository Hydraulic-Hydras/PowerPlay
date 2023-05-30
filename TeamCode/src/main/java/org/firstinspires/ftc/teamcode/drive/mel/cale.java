package org.firstinspires.ftc.teamcode.drive.mel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class cale extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        // variables for drive train control
        float vertical;
        float horizontal;
        float pivot;

        double leftRearSpeed;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double rightRearSpeed;

        double powerMultiplier = 1;

        // drivetrain hardware
        DcMotor leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        DcMotor rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        DcMotor leftrear = hardwareMap.get(DcMotor.class, "leftrear");
        DcMotor rightrear = hardwareMap.get(DcMotor.class, "rightrear");

        // lifting motor
        DcMotor rightlift = hardwareMap.get(DcMotor.class, "rightlift");
        DcMotor leftlift = hardwareMap.get(DcMotor.class, "leftlift");

        // turret
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");

        // claw
        Servo intake = hardwareMap.get(Servo.class, "intake");

        //initialization
        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftrear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightrear.setDirection(DcMotorSimple.Direction.FORWARD);

        //lift
        leftlift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightlift.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        //brakes
        leftrear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightrear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {

            // mecanum drivetrain control

            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            rightFrontSpeed = ((vertical - horizontal) - pivot);
            rightRearSpeed = ((vertical + horizontal) - pivot);
            leftFrontSpeed = (vertical + horizontal + pivot);
            leftRearSpeed = ((vertical - horizontal) + pivot);

            leftfront.setPower(leftFrontSpeed * powerMultiplier);
            rightfront.setPower(rightFrontSpeed * powerMultiplier);
            leftrear.setPower(leftRearSpeed * powerMultiplier);
            rightrear.setPower(rightRearSpeed * powerMultiplier);

            // servo
            if (gamepad1.a) {
                intake.setPosition(0.2);
            } else if (gamepad1.b) {
                intake.setPosition(1);
            }

            // lifts up n down
            if (gamepad2.left_trigger > 0) {
                rightlift.setPower(1);
                leftlift.setPower(1);
            } else if (gamepad2.right_trigger > 0 ) {
                rightlift.setPower(-0.5);
                leftlift.setPower(-0.5);
            }   else {
                leftlift.setPower(0);
                rightlift.setPower(0);
            }


        }
    }
}
