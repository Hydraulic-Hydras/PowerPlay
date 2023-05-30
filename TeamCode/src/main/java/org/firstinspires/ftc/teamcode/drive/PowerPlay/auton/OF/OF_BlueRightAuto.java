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
public class OF_BlueRightAuto extends LinearOpMode {

    OpenCvCamera camera;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public int finalx; // 35(2), 12(1), 58(3)
    public int finaly; // 12(2), (12), 12(3)
    public int finalh; // 90

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

        // Intake and Turret hardwareMap
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        Servo CLAW = hardwareMap.get(Servo.class, "CLAW");

        // Set motor directions
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_L.setDirection(DcMotorSimple.Direction.FORWARD);
        lift_R.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set ZERO POWER BEHAVIOR
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        telemetry.setMsTransmissionInterval(50);

        // ENCODERS
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // CLAW POSITIONS
        double open = 0.5;
        double grab = 1;

        CLAW.setPosition(open);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle || tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)

                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), DriveConstants.TRACK_WIDTH))

                // PRE LOAD
                .UNSTABLE_addTemporalMarkerOffset(0, () -> CLAW.setPosition(grab))
                // Wait a few milliseconds to remove servo delay
                .waitSeconds(0.7)
                .forward(50)

                .addTemporalMarker(0.9, () -> {
                    lift_L.setTargetPosition(1900);
                    lift_L.setPower(0.78);
                    lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift_R.setTargetPosition(1900);
                    lift_R.setPower(0.78);
                    lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                })

                .resetConstraints()

                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-21, 15, Math.toRadians(270)))

                .UNSTABLE_addTemporalMarkerOffset(1, () -> CLAW.setPosition(open))

                .waitSeconds(0.5)
                .resetConstraints()

                .build();

        // REVERSE THE AXIS
        if(tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            finalx = -34;
            finaly = 17;
            finalh = 270;
            telemetry.addLine("middle");
            telemetry.update();
        }else if (tagOfInterest.id == left){
            //right code
            finalx = -10;
            finaly = 17;
            finalh = 270;
            telemetry.addLine("left");
            telemetry.update();
        }else if (tagOfInterest.id == right){
            //left code
            finalx = -60;
            finaly = 16;
            finalh = 0;
            telemetry.addLine("right");
            telemetry.update();
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(preload.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(finalx,finaly,Math.toRadians(finalh)))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(preload);
        drive.followTrajectorySequence(park);

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry  */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            telemetry.addLine("middle");
            telemetry.update();
        }else if (tagOfInterest.id == left){
            //left code
            telemetry.addLine("left");
            telemetry.update();
        }else if (tagOfInterest.id == right){
            //right code
            telemetry.addLine("right");
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        telemetry.update();
    }
}
