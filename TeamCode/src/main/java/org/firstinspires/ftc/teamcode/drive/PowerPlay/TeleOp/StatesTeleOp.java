package org.firstinspires.ftc.teamcode.drive.PowerPlay.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Created by Tea on 2/14/2023

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class StatesTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        float vertical;
        float horizontal;
        float pivot;

        double leftRearSpeed;
        double leftFrontSpeed;
        double rightFrontSpeed;
        double rightRearSpeed;

        double powerMultiplier = 1;

        // DriveTrain hardware
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Cascade lifts
        DcMotor lift_L = hardwareMap.get(DcMotor.class, "lift_L");
        DcMotor lift_R = hardwareMap.get(DcMotor.class, "lift_R");

        // Module hardwareMap
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        Servo CLAW = hardwareMap.get(Servo.class, "CLAW");

        TouchSensor high_limit = hardwareMap.get(TouchSensor.class, "high_limit");
        TouchSensor low_limit = hardwareMap.get(TouchSensor.class, "low_limit");
        DistanceSensor Cone_sensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Cone_sensor");

        // LED lights
        DigitalChannel LEDL_Red = hardwareMap.get(DigitalChannel.class, "LED-L_Red");
        DigitalChannel LEDL_Green = hardwareMap.get(DigitalChannel.class, "LED-L_Green");
        DigitalChannel LEDR_Red = hardwareMap.get(DigitalChannel.class, "LED-R_Red");
        DigitalChannel LEDR_Green = hardwareMap.get(DigitalChannel.class, "LED-R_Green");


        // Put initialization blocks here.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_L.setDirection(DcMotorSimple.Direction.FORWARD);
        lift_R.setDirection(DcMotorSimple.Direction.FORWARD);


        LEDL_Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDL_Green.setMode(DigitalChannel.Mode.OUTPUT);
        LEDR_Red.setMode(DigitalChannel.Mode.OUTPUT);
        LEDR_Green.setMode(DigitalChannel.Mode.OUTPUT);

        // Set ZERO POWER BEHAVIOR
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (Cone_sensor_DistanceSensor.getDistance(DistanceUnit.MM) >= 6 && Cone_sensor_DistanceSensor.getDistance(DistanceUnit.MM) <= 56) {
                    LEDL_Red.setState(true);
                    LEDL_Green.setState(false);
                    LEDR_Red.setState(true);
                    LEDR_Green.setState(false);
                } else {
                    LEDL_Red.setState(false);
                    LEDL_Green.setState(true);
                    LEDR_Red.setState(false);
                    LEDR_Green.setState(true);
                }

                // Put loop blocks here.
                // Mecanum Wheel Drivetrain control
                vertical = -gamepad1.left_stick_y;
                horizontal = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;
                rightFrontSpeed = ((vertical - horizontal) - pivot);
                rightRearSpeed = ((vertical + horizontal) - pivot);
                leftFrontSpeed = (vertical + horizontal + pivot);
                leftRearSpeed = ((vertical - horizontal) + pivot);

                leftFront.setPower(leftFrontSpeed * powerMultiplier);
                rightFront.setPower(rightFrontSpeed * powerMultiplier);
                leftRear.setPower(leftRearSpeed * powerMultiplier);
                rightRear.setPower(rightRearSpeed * powerMultiplier);

                // Change DriveTrain speed
                // Left bumper speed to strafe and have more control
                if (gamepad1.left_bumper) {
                    powerMultiplier = 0.5;
                } else {
                    powerMultiplier = 1.0;
                }

                // CLAW
                // A/CROSS  = OPEN
                // B/CIRCLE = Close
                if (gamepad1.a) {
                    CLAW.setPosition(0.2);
                } else if (gamepad1.b) {
                    CLAW.setPosition(1);
                }

                // Cascade Lifts Left and Right
                // Right trigger to go up
                // Left trigger to come down
                if (gamepad2.left_trigger > 0 && !low_limit.isPressed()) {
                    lift_L.setPower(gamepad2.left_trigger * -0.65);
                    lift_R.setPower(gamepad2.left_trigger * -0.65);
                }   else if (gamepad2.right_trigger > 0 && !high_limit.isPressed()) {
                    lift_L.setPower(gamepad2.right_trigger * 1);
                    lift_R.setPower(gamepad2.right_trigger * 1);
                }   else {
                    lift_L.setPower(0);
                    lift_R.setPower(0);
                }


                // TURRET SYSTEM FOR DRIVER
                // Right Trigger to turn CLOCKWISE (Speed 75%)
                // Left Trigger to turn ANTI-CLOCKWISE (Speed 75%)

                // Turret System for CO DRIVER
                // Right Bumper to turn turret CLOCKWISE (Speed 65%)
                // Left Bumper to turn turret ANTI-CLOCKWISE (Speed 65%)
                if (gamepad2.right_bumper) {
                    turret.setPower(-0.65);
                } else if (gamepad2.left_bumper) {
                    turret.setPower(0.65);
                } else if (gamepad1.right_trigger > 0) {
                    turret.setPower(gamepad1.right_trigger * 0.75);
                } else if (gamepad1.left_trigger > 0) {
                    turret.setPower(gamepad1.left_trigger * -0.75);
                } else {
                    turret.setPower(0);
                }

                telemetry.addData("high_limit", high_limit.getValue());
                telemetry.addData("low_limit", low_limit.getValue());
                telemetry.addData("lift_L", lift_L.getCurrentPosition());
                telemetry.addData("lift_R", lift_R.getCurrentPosition());
                telemetry.addData("CLAW", CLAW.getPosition());
                telemetry.addData("turret", turret.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}