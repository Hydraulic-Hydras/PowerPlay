package org.firstinspires.ftc.teamcode.drive.PowerPlay.auton.SpeedTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "15:1 time", preselectTeleOp = "StatesTeleOp")
public class DriveTrainauto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Motor Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // zero power
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Starting pose
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startpose = new Pose2d(35, -64, Math.toRadians(90));

        drive.setPoseEstimate(startpose);


        TrajectorySequence motion = drive.trajectorySequenceBuilder(startpose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(100, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(100))

                // RED RIGHT SIDE
                // pre load
                .splineTo(new Vector2d(36.67, -30.50), Math.toRadians(106.73))
                .splineTo(new Vector2d(31.17, -7.17), Math.toRadians(136.17))
                .waitSeconds(1)

                // one cycle
                .setReversed(true)
                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                .waitSeconds(1)

                .setReversed(false)
                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                .waitSeconds(1)

                .build();


        waitForStart();
        if (isStopRequested()) return;
        // DRIVE AUTONOMOUS
        drive.followTrajectorySequence(motion);
    }
}
