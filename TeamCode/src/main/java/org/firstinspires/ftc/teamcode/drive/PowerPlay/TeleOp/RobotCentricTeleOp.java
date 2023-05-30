package org.firstinspires.ftc.teamcode.drive.PowerPlay.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobotCentricTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        double powerMultiplier = 1;

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Directions ( NOT CORRECT)
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double leftFrontSpeed = (y + x + rx) / denominator;
                double leftRearSpeed = (y - x + rx) / denominator;
                double rightFrontSpeed = (y - x - rx) / denominator;
                double rightRearSpeed = (y + x - rx) / denominator;

                leftFront.setPower(leftFrontSpeed * powerMultiplier);
                leftRear.setPower(leftRearSpeed * powerMultiplier);
                rightFront.setPower(rightFrontSpeed * powerMultiplier);
                rightRear.setPower(rightRearSpeed * powerMultiplier);

                // Change DriveTrain speed
                // Left bumper speed to strafe and have more control
                if (gamepad1.left_bumper) {
                    powerMultiplier = 0.5;
                } else {
                    powerMultiplier = 1.0;
                }

            }
        }
    }
}
