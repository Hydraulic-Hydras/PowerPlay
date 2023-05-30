package org.firstinspires.ftc.teamcode.drive.PowerPlay.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous
public class finishedENC extends LinearOpMode {

    // BLUE RIGHT SIDE
    DcMotor lift_L;
    DcMotor lift_R;
    DcMotor turret;
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

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive Train hardwareMap
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // Modules hardwareMap
        Servo CLAW = hardwareMap.get(Servo.class, "CLAW");
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");

        // Cascades
        DcMotor lift_L = hardwareMap.get(DcMotor.class,"lift_L");
        DcMotor lift_R = hardwareMap.get(DcMotor.class, "lift_R");


        // Set motor directions
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_L.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_R.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set ZERO POWER BEHAVIOR
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CAMERA
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

        telemetry.setMsTransmissionInterval(50);

        // POSE
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        // ENCODERS
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // CLAW POSITIONS
        double open = 0.2;
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


        // CYCLES
        TrajectorySequence motion = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))

                // PRE-LOAD
                .UNSTABLE_addTemporalMarkerOffset(0, () -> CLAW.setPosition(grab))

                .addTemporalMarker(0, () -> moveTurret(-63, 0.55))

                .waitSeconds(0.8)

                .lineToLinearHeading(new Pose2d(-35,15, Math.toRadians(-60)))

                .addTemporalMarker(1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .waitSeconds(0.1)

                .splineTo(new Vector2d(-27, 6), Math.toRadians(-60),
                        SampleMecanumDrive.getVelocityConstraint(30,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    moveLift_L(1500, 0.85);
                    moveLift_R(1500, 0.85);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> CLAW.setPosition(open))

                // CYCLE 1
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-35, 18))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(304, 0.80);
                    moveLift_R(304, 0.80);
                })

                .splineToSplineHeading(new Pose2d(-60, 14, Math.toRadians(180)), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-25, 14.5),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .UNSTABLE_addTemporalMarkerOffset(0, () ->  moveTurret(-986, 0.70))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> moveTurret(-63, 0.75))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(284, 0.80);
                    moveLift_R(284, 0.80);
                })

                // CYCLE 2
                .lineTo(new Vector2d(-60, 14),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-25,14.5),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> moveTurret(-986, 0.70))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> moveTurret(-63, 0.75))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(230, 0.80);
                    moveLift_R(230, 0.80);
                })

                // CYCLE 3
                .lineTo(new Vector2d(-60, 14),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-25,14.5),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> moveTurret(-986, 0.70))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> moveTurret(-63, 0.75))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(230, 0.80);
                    moveLift_R(230, 0.80);
                })

                // CYCLE 4
                .lineTo(new Vector2d(-60, 14),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-25,14.5),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> moveTurret(-986, 0.70))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> moveTurret(-63, 0.75))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(230, 0.80);
                    moveLift_R(230, 0.80);
                })

                // CYCLE 5
                .lineTo(new Vector2d(-60, 14),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(50))

                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> CLAW.setPosition(grab))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    moveLift_L(1970, 1);
                    moveLift_R(1970, 1);
                })

                .lineTo(new Vector2d(-25,14.5),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> moveTurret(-986, 0.70))

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> CLAW.setPosition(open))

                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> moveTurret(-63, 0.75))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    moveLift_L(230, 0.80);
                    moveLift_R(230, 0.80);
                })

                .build();

        // PARK VALUES
        if(tagOfInterest == null || tagOfInterest.id == middle) {
            //middle code
            finalx = -34;
            finaly = 18;
            finalh = 270;
            telemetry.addLine("middle");
            telemetry.update();
        }else if (tagOfInterest.id == right){
            //right code
            finalx = -10;
            finaly = 18;
            finalh = 270;
            telemetry.addLine("right");
            telemetry.update();
        }else if (tagOfInterest.id == left){
            //left code
            finalx = -60;
            finaly = 18;
            finalh = 270;
            telemetry.addLine("left");
            telemetry.update();
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(motion.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(finalx,finaly,Math.toRadians(finalh)))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        // DRIVE AUTONOMOUS
        drive.followTrajectorySequence(motion);
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

    // APRILTAG TELEMETRY FOR SCAN
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

    // AUTONOMOUS FUNCTIONS
    public void moveLift_L (int pos, double power) {
        lift_L.setTargetPosition(pos);
        lift_L.setPower(power);
        lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public  void moveLift_R (int pos, double power) {
        lift_R.setTargetPosition(pos);
        lift_R.setPower(power);
        lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveTurret (int pos, double power) {
        turret.setTargetPosition(pos);
        turret.setPower(power);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
