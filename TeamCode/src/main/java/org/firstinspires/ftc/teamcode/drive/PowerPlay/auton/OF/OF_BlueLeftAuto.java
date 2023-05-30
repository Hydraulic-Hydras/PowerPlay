package org.firstinspires.ftc.teamcode.drive.PowerPlay.auton.OF;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlay.Hardware.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous
public class OF_BlueLeftAuto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public int finalx;
    public int finaly;
    public int finalh;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    // tag ID's of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        // Drive Train hardwareMap
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Cascade lift hardwareMap
        DcMotor lift_L = hardwareMap.get(DcMotor.class, "lift_L");
        DcMotor lift_R = hardwareMap.get(DcMotor.class, "lift_R");

        // CLAW hardwareMap
        Servo CLAW = hardwareMap.get(Servo.class, "CLAW");


        // Set motor directions
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        lift_L.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_R.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set ZERO POWER BEHAVIOR
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35, 64, Math.toRadians(-90)); // (35, 61, -90)

        drive.setPoseEstimate(startPose);

        telemetry.setMsTransmissionInterval(50);


        // ENCODERS
        lift_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // CLAW POSITIONS
        double open = 0.3;
        double grab = 1;

        CLAW.setPosition(open);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == left || tag.id == middle || tag.id == right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))

                // Wait for Claw to grab
                .UNSTABLE_addTemporalMarkerOffset(0, () -> CLAW.setPosition(grab))
                .waitSeconds(0.9)

                .addTemporalMarker(1.2, () -> {
                    lift_L.setTargetPosition(1970);
                    lift_L.setPower(0.9);
                    lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift_R.setTargetPosition(1970);
                    lift_R.setPower(0.9);
                    lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })

                .forward(50)
                .waitSeconds(0.1)
                .turn(Math.toRadians(-42))

                .resetConstraints()

                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(17, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> CLAW.setPosition(open))
                .waitSeconds(0.3)

                .back(9)
                .resetConstraints()

                .build();

        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(preload.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(58))

                .addTemporalMarker(0, () -> {
                    lift_L.setTargetPosition(365);
                    lift_L.setPower(0.65);
                    lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift_R.setTargetPosition(310);
                    lift_R.setPower(0.65);
                    lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })

                .turn(Math.toRadians(131))
                .forward(23)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift_L.setTargetPosition(1970);
                    lift_L.setPower(1);
                    lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift_R.setTargetPosition(1970);
                    lift_R.setPower(1);
                    lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })

                .waitSeconds(1)
                .back(23)
                .turn(Math.toRadians(-123))

                .resetConstraints()

                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(17, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))
                .waitSeconds(0.7)
                .back(9)

                .resetConstraints()

                .build();


        // 12.3 seconds of all (.waitSecond) commands which leaves me with 17.7 seconds to score and park
        // Will reduce the time between actions

        if (tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            finalx = 37;
            finaly = 13;
            finalh = 270;
            telemetry.addLine("middle");
            telemetry.update();
        } else if (tagOfInterest.id == right) {
            //right code
            finalx = 9;
            finaly = 17;
            finalh = 270;
            telemetry.addLine("right");
            telemetry.update();
        } else if (tagOfInterest.id == left) {
            //left code
            finalx = 62;
            finaly = 16;
            finalh = 270;
            telemetry.addLine("left");
            telemetry.update();
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(cycle1.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(finalx, finaly, Math.toRadians(finalh)))
                .build();


        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(preload);
        drive.followTrajectorySequence(cycle1);
        drive.followTrajectorySequence(park);

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry  */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            telemetry.addLine("middle");
            telemetry.update();
        } else if (tagOfInterest.id == left) {
            //left code
            telemetry.addLine("left");
            telemetry.update();
        } else if (tagOfInterest.id == right) {
            //right code
            telemetry.addLine("right");
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();
    }
}