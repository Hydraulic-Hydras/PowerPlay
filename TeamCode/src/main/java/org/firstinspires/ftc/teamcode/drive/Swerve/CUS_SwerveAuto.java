package org.firstinspires.ftc.teamcode.drive.Swerve;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlay.Hardware.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class CUS_SwerveAuto extends LinearOpMode {

    // BLUE LEFT SIDE
    DcMotor lift_L;
    DcMotor lift_R;
    DcMotor turret;
    Servo CLAW;

    // CLAW POSITIONS
    double open = 0.2;
    double grab = 1;

    Robot robot;

    OpenCvCamera camera;

    // tag ID's of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;

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

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;


    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();

        // MOTOR DIRECTIONS
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        lift_L.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_R.setDirection(DcMotorSimple.Direction.REVERSE);

        // ZERO POWER BEHAVIOUR
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // ENCODERS
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        CLAW.setPosition(open);

        // rotate module to face forward
        robot.driveController.rotateModules(Vector2d.FORWARD, true, 0, this);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {

            // PRE-LOAD
            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_L(1970, 1);
            moveLift_R(1970, 1);

            // drive forward 190.5 cm ~ 75 inches
            robot.driveController.drive(Vector2d.FORWARD, 190.5, 1, this);

            // turret to the right
            moveTurret(986, 0.65);

            // drop cone
            CLAW.setPosition(open);


            // CYCLE 1
            // lift down
            moveLift_L(329, 0.8);
            moveLift_R(329, 0.8);

            // rotate module to face backwards
            robot.driveController.rotateModules(Vector2d.BACKWARD, true, 0, this);

            // drive backwards 43.18 cm ~ 17 inches
            robot.driveController.drive(Vector2d.BACKWARD, 43.18, 1, this);

            // rotate module to face left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            // turret to the left
            moveTurret(-1103, 1);

            // drive left 76.2 cm ~ 30 inches
            robot.driveController.drive(Vector2d.LEFT, 76.2, 1, this);

            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            // rotate modules to face right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // drive right 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.RIGHT, 101.6, 1, this);

            // turret to original pos
            moveTurret(63, 0.65);

            // drop cone
            CLAW.setPosition(open);


            // CYCLE 2
            // move turret to left
            moveTurret(-1103, 1);

            // lift down
            moveLift_L(289, 1);
            moveLift_R(289, 1);

            // rotate module to face left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            // drive left 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.LEFT, 101.6, 1, this);

            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            // rotate modules to face right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // drive right 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.RIGHT, 101.6, 1, this);

            // turret to original pos
            moveTurret(63, 0.65);

            // drop cone
            CLAW.setPosition(open);


            // CYCLE 3
            // move turret to left
            moveTurret(-1103, 1);

            // lift down
            moveLift_L(243, 1);
            moveLift_R(243, 1);

            // rotate module to face left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            // drive left 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.LEFT, 101.6, 1, this);

            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            // rotate modules to face right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // drive right 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.RIGHT, 101.6, 1, this);

            // turret to original pos
            moveTurret(63, 0.65);

            // drop cone
            CLAW.setPosition(open);


            // CYCLE 4
            // move turret to left
            moveTurret(-1103, 1);

            // lift down
            moveLift_L(203, 1);
            moveLift_R(203, 1);

            // rotate module to face left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            // drive left 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.LEFT, 101.6, 1, this);

            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            // rotate modules to face right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // drive right 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.RIGHT, 101.6, 1, this);

            // turret to original pos
            moveTurret(63, 0.65);

            // drop cone
            CLAW.setPosition(open);


            // CYCLE 5
            // move turret to left
            moveTurret(-1103, 1);

            // lift down
            moveLift_L(180, 1);
            moveLift_R(180, 1);

            // rotate module to face left
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);

            // drive left 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.LEFT, 101.6, 1, this);

            // grab cone
            CLAW.setPosition(grab);

            // lift up
            moveLift_R(1930, 1);
            moveLift_L(1930, 1);

            // rotate modules to face right
            robot.driveController.rotateModules(Vector2d.RIGHT, true, 0, this);

            // drive right 101.6 cm ~ 40 inches
            robot.driveController.drive(Vector2d.RIGHT, 101.6, 1, this);

            // turret to original pos
            moveTurret(63, 0.65);

            // drop cone
            CLAW.setPosition(open);

            // lift down
            moveLift_R(63,1);
            moveLift_L(63,1);

        }

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

        // DETECTION
        {
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

        waitForStart();
        if (isStopRequested()) return;

        // PARKING
        if ((tagOfInterest == null) | (tagOfInterest.id == middle)) {
            // middle code
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);
            robot.driveController.drive(Vector2d.LEFT, 38.1, 1, this);

            telemetry.addLine("middle");
            telemetry.update();

        }   else if (tagOfInterest.id == right) {
            // right code
            robot.driveController.rotateModules(Vector2d.BACKWARD, true, 0 ,this);
            robot.driveController.drive(Vector2d.BACKWARD, 20, 1, this);

            telemetry.addLine("right");
            telemetry.update();

        }   else if (tagOfInterest.id == left) {
            // left code
            robot.driveController.rotateModules(Vector2d.LEFT, true, 0, this);
            robot.driveController.drive(Vector2d.LEFT, 114.3, 1, this);

            telemetry.addLine("left");
            telemetry.update();
        }

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

    // APRILTAG TELEMETRY FOR SCAN
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

            // AUTONOMOUS FUNCTIONS
            public void moveLift_L ( int pos, double power){
                lift_L.setTargetPosition(pos);
                lift_L.setPower(power);
                lift_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            public void moveLift_R ( int pos, double power){
                lift_R.setTargetPosition(pos);
                lift_R.setPower(power);
                lift_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            public void moveTurret ( int pos, double power){
                turret.setTargetPosition(pos);
                turret.setPower(power);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }