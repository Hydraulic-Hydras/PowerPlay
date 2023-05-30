package org.firstinspires.ftc.teamcode.drive.Swerve;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlay.Hardware.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class TRAJ_SwerveAuto extends LinearOpMode {

    private List<TemporalMarker> temporalMarkers;
    private double currentDuration;


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
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();

        // Modules hardwareMap
        Servo CLAW = hardwareMap.get(Servo.class, "CLAW");

        // MOTOR DIRECTIONS
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_L.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_R.setDirection(DcMotorSimple.Direction.REVERSE);

        // ZERO POWER BEHAVIOUR
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

        // STARTING POSE
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

            // CYCLES HERE
            // GRAB
            CLAW.setPosition(grab);

            // 1 inch = 2.54cm
            // 10 inches = 25.4cm
            // 20 inches = 50.8cm
            // 30 inches = 76.2cm
            // 40 inches = 101.6cm
            // 50 inches = 127cm
            // 60 inches = 152.4cm


            //rotate modules to face Forward
            robot.driveController.rotateModules(Vector2d.FORWARD, true, 0.1, this);

            //drive 152.4cm (60 inches) straight (while facing forward)
            robot.driveController.drive(Vector2d.FORWARD, 152.4, 1, this);

            //turn to face robot left 45 degrees
            robot.driveController.rotateRobot(Angle.HALF_LEFT, this);

            // Lift up
            addTemporalMarker(1, () -> {
                moveLift_L(1970, 1);
                moveLift_R(1970, 1);
            });

            //drive 25.4cm (10 inches) straight (while facing forward)
            robot.driveController.drive(Vector2d.FORWARD, 25.4, 0.8, this);

            // OPEN
            CLAW.setPosition(open);

            //rotate modules to face backwards
            robot.driveController.rotateModules(Vector2d.BACKWARD, true, 0, this);

            // pos robot straight
            robot.driveController.rotateRobot(Angle.FORWARD, this);

            //drive 25.4cm (10 inches) backwards (while facing forward)
            robot.driveController.drive(Vector2d.BACKWARD, 25.4, 0.6, this);

            // Lift down
            UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                moveLift_L(230, 1);
                moveLift_R(230, 1);
            });

            //rotate modules to face to the Right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // Turret placement
            moveTurret(900, 0.80);

            //drive 76.2cm (30 inches) right (while facing forward)
            robot.driveController.drive(Vector2d.RIGHT, 76.2, 0.8, this);

            // GRAB
            CLAW.setPosition(grab);

            // Lift Up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            //rotate modules to face to the left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            moveTurret(63, 0.60);

            //drive 101.6cm (40 inches) left (while facing forward)
            robot.driveController.drive(Vector2d.LEFT, 101.6, 1, this);

            //rotate modules to face to the right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // OPEN
            CLAW.setPosition(open);


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

        Trajectory park = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(finalx,finaly,Math.toRadians(finalh)))
                .build();

        waitForStart();
        if (isStopRequested()) return;
        // DRIVE AUTONOMOUS
        drive.followTrajectory(park);

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
    public void moveLift_L(int pos, int power) {
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

    public TRAJ_SwerveAuto addTemporalMarker(MarkerCallback callback) {
        return this.addTemporalMarker(currentDuration, callback);
    }

    public TRAJ_SwerveAuto UNSTABLE_addTemporalMarkerOffset(double offset, MarkerCallback callback) {
        return this.addTemporalMarker(currentDuration + offset, callback);
    }

    public TRAJ_SwerveAuto addTemporalMarker(double time, MarkerCallback callback) {
        return this.addTemporalMarker(0.0, time, callback);
    }

    public TRAJ_SwerveAuto addTemporalMarker(double scale, double offset, MarkerCallback callback) {
        return this.addTemporalMarker(time -> scale * time + offset, callback);
    }

    public TRAJ_SwerveAuto addTemporalMarker(TimeProducer time, MarkerCallback callback) {
        this.temporalMarkers.add(new TemporalMarker(time, callback));
        return this;
    }

    public void setTemporalMarkers(List<TemporalMarker> temporalMarkers) {
        this.temporalMarkers = temporalMarkers;
    }
}
