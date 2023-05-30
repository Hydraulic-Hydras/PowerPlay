package org.firstinspires.ftc.teamcode.drive.SwerveTrajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.acmerobotics.roadrunner.trajectory.DisplacementProducer;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.SpatialMarker;
import com.acmerobotics.roadrunner.trajectory.TemporalMarker;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Swerve.DriveController;
import org.firstinspires.ftc.teamcode.drive.Swerve.DriveModule;
import org.firstinspires.ftc.teamcode.drive.Swerve.Robot;
import org.firstinspires.ftc.teamcode.drive.Swerve.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments.SwerveSequenceSegment;
import org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments.TrajSeg_swerve;
import org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments.TurnSegment_swerve;
import org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments.WaitSegment_swerve;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public abstract class TrajSeqBuild_swerve extends LinearOpMode {

    Robot robot;
    DriveModule moduleLeft;
    DriveModule moduleRight;

    DriveController driveController;

    //used for straight line distance tracking
    double robotDistanceTraveled = 0;
    double previousRobotDistanceTraveled = 0;
    double moduleLeftLastDistance;
    double moduleRightLastDistance;

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 2;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;

    public final double resolution = 0.25;
    private final TrajectoryVelocityConstraint baseVelConstraint;
    private final TrajectoryAccelerationConstraint baseAccelConstraint;
    private TrajectoryVelocityConstraint currentVelConstraint;
    private TrajectoryAccelerationConstraint currentAccelConstraint;
    private final List<SwerveSequenceSegment> swerveSequenceSegments;
    private final List<TemporalMarker> temporalMarkers;
    private final List<DisplacementMarker> displacementMarkers;
    private final List<SpatialMarker> spatialMarkers;
    private Pose2d lastPose;
    private double tangentOffset;
    private boolean setAbsoluteTangent;
    private double absoluteTangent;
    private TrajectoryBuilder currentTrajectoryBuilder;
    private double currentDuration;
    private double currentDisplacement;
    private double lastDurationTraj;
    private double lastDisplacementTraj;

    public TrajSeqBuild_swerve(
            Pose2d startPose,
            Double startTangent,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {

        robot = new Robot(this, true);
        robot.initIMU();

        this.baseVelConstraint = baseVelConstraint;
        this.baseAccelConstraint = baseAccelConstraint;

        this.currentVelConstraint = baseVelConstraint;
        this.currentAccelConstraint = baseAccelConstraint;

        swerveSequenceSegments = new ArrayList<>();

        temporalMarkers = new ArrayList<>();
        displacementMarkers = new ArrayList<>();
        spatialMarkers = new ArrayList<>();

        lastPose = startPose;

        tangentOffset = 0.0;

        setAbsoluteTangent = (startTangent != null);
        absoluteTangent = startTangent != null ? startTangent : 0.0;

        currentTrajectoryBuilder = null;

        currentDuration = 0.0;
        currentDisplacement = 0.0;

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;
    }

    public TrajSeqBuild_swerve(
            Pose2d startPose,
            TrajectoryVelocityConstraint baseVelConstraint,
            TrajectoryAccelerationConstraint baseAccelConstraint,
            double baseTurnConstraintMaxAngVel,
            double baseTurnConstraintMaxAngAccel
    ) {
        this(
                startPose, null,
                baseVelConstraint, baseAccelConstraint,
                baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel
        );
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
    }


    // FORWARD
    public TrajSeqBuild_swerve forward(double distance) {
        robot.driveController.rotateModules(Vector2d.FORWARD, true, 0.1, this);
        robot.driveController.drive(Vector2d.FORWARD, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.forward(distance, currentVelConstraint, currentAccelConstraint));
    }

    public TrajSeqBuild_swerve forward(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        robot.driveController.rotateModules(Vector2d.FORWARD, true, 0.1, this);
        robot.driveController.drive(Vector2d.FORWARD, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.forward(distance, velConstraint, accelConstraint));
    }

    // BACK
    public TrajSeqBuild_swerve back(double distance) {
        robot.driveController.rotateModules(Vector2d.BACKWARD, true, 0.1, this);
        robot.driveController.drive(Vector2d.BACKWARD, distance, 1, this);
        return  addPath(() -> currentTrajectoryBuilder.back(distance, currentVelConstraint, currentAccelConstraint));
    }

    public TrajSeqBuild_swerve back(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        robot.driveController.rotateModules(Vector2d.BACKWARD, true, 0.1, this);
        robot.driveController.drive(Vector2d.BACKWARD, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.back(distance, velConstraint, accelConstraint));
    }

    // LEFT
    public TrajSeqBuild_swerve strafeLeft(double distance) {
        robot.driveController.rotateModules(Vector2d.LEFT, true, 0.1, this);
        robot.driveController.drive(Vector2d.LEFT, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, currentVelConstraint, currentAccelConstraint));
    }

    public TrajSeqBuild_swerve strafeLeft(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        robot.driveController.rotateModules(Vector2d.LEFT, true, 0.1, this);
        robot.driveController.drive(Vector2d.LEFT, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.strafeLeft(distance, velConstraint, accelConstraint));
    }


    // RIGHT
    public TrajSeqBuild_swerve strafeRight(double distance) {
        robot.driveController.rotateModules(Vector2d.RIGHT, true, 0.1, this);
        robot.driveController.drive(Vector2d.RIGHT, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, currentVelConstraint, currentAccelConstraint));
    }

    public TrajSeqBuild_swerve strafeRight(
            double distance,
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        robot.driveController.rotateModules(Vector2d.RIGHT, true, 0.1, this);
        robot.driveController.drive(Vector2d.RIGHT, distance, 1, this);
        return addPath(() -> currentTrajectoryBuilder.strafeRight(distance, velConstraint, accelConstraint));
    }


    // PATH CALLBACK METHOD
    private TrajSeqBuild_swerve addPath(AddPathCallBack callBack) {
        if (currentTrajectoryBuilder == null) newPath();

        try {
            callBack.run();
        } catch (PathContinuityViolationException e) {
            newPath();
            callBack.run();
        }

        Trajectory builtTraj = currentTrajectoryBuilder.build();

        double durationDifference = builtTraj.duration() - lastDurationTraj;
        double displacementDifference = builtTraj.getPath().length() - lastDisplacementTraj;

        lastPose = builtTraj.end();
        currentDuration += durationDifference;
        currentDisplacement += displacementDifference;

        lastDurationTraj = builtTraj.duration();
        lastDisplacementTraj = builtTraj.getPath().length();

        return this;
    }

    private void pushPath() {
        if (currentTrajectoryBuilder != null) {
            Trajectory builtTraj = currentTrajectoryBuilder.build();
            swerveSequenceSegments.add(new TrajSeg_swerve(builtTraj));
        }

        currentTrajectoryBuilder = null;
    }

    private void newPath() {
        if (currentTrajectoryBuilder != null)
            pushPath();

        lastDurationTraj = 0.0;
        lastDisplacementTraj = 0.0;

        double tangent = setAbsoluteTangent ? absoluteTangent : Angle.norm(lastPose.getHeading() + tangentOffset);

        currentTrajectoryBuilder = new TrajectoryBuilder(lastPose, tangent, currentVelConstraint, currentAccelConstraint, resolution);
    }


    // TANGENT VALUES
    public TrajSeqBuild_swerve setTangent(double tangent) {
        setAbsoluteTangent = true;
        absoluteTangent = tangent;

        pushPath();

        return this;
    }

    private TrajSeqBuild_swerve setTangentOffset(double offset) {
        setAbsoluteTangent = false;

        this.tangentOffset = offset;
        this.pushPath();

        return this;
    }

    // TIME MARKERS
    public TrajSeqBuild_swerve addTemporalMarker(MarkerCallback callback) {
        return this.addTemporalMarker(currentDuration, callback);
    }

    public TrajSeqBuild_swerve UNSTABLE_addTemporalMarkerOffset(double offset, MarkerCallback callback) {
        return this.addTemporalMarker(currentDuration + offset, callback);
    }

    public TrajSeqBuild_swerve addTemporalMarker(double time, MarkerCallback callback) {
        return this.addTemporalMarker(0.0, time, callback);
    }

    public TrajSeqBuild_swerve addTemporalMarker(double scale, double offset, MarkerCallback callback) {
        return this.addTemporalMarker(time -> scale * time + offset, callback);
    }

    public TrajSeqBuild_swerve addTemporalMarker(TimeProducer time, MarkerCallback callback) {
        this.temporalMarkers.add(new TemporalMarker(time, callback));
        return this;
    }

    // DONT EVEN KNOW WHAT THESE DO BUT YEA
    public TrajSeqBuild_swerve addSpatialMarker(com.acmerobotics.roadrunner.geometry.Vector2d point, MarkerCallback callback) {
        this.spatialMarkers.add(new SpatialMarker(point, callback));
        return this;
    }


    // DISTANCE MARKERS
    public TrajSeqBuild_swerve addDisplacementMarker(MarkerCallback callback) {
        return this.addDisplacementMarker(currentDisplacement, callback);
    }

    public TrajSeqBuild_swerve UNSTABLE_addDisplacementMarkerOffset(double offset, MarkerCallback callback) {
        return this.addDisplacementMarker(currentDisplacement + offset, callback);
    }

    public TrajSeqBuild_swerve addDisplacementMarker(double displacement, MarkerCallback callback) {
        return this.addDisplacementMarker(0.0, displacement, callback);
    }

    public TrajSeqBuild_swerve addDisplacementMarker(double scale, double offset, MarkerCallback callback) {
        return addDisplacementMarker((displacement -> scale * displacement + offset), callback);
    }

    public TrajSeqBuild_swerve addDisplacementMarker(DisplacementProducer displacement, MarkerCallback callback) {
        displacementMarkers.add(new DisplacementMarker(displacement, callback));

        return this;
    }


    // WAIT COMMAND
    public TrajSeqBuild_swerve waitSeconds(double seconds) {
        pushPath();
        swerveSequenceSegments.add(new WaitSegment_swerve(lastPose, seconds, Collections.emptyList()));

        currentDuration += seconds;
        return this;
    }

    // TRAJECTORIES
    public TrajSeqBuild_swerve addTrajectory(Trajectory trajectory) {
        pushPath();

        swerveSequenceSegments.add(new TrajSeg_swerve(trajectory));
        return this;
    }

    public TrajSeq_swerve build() {
        pushPath();

        List<TrajectoryMarker> globalMarkers = convertMarkersToGlobal(
                swerveSequenceSegments,
                temporalMarkers, displacementMarkers, spatialMarkers
        );

        return new TrajSeq_swerve(projectGlobalMarkersToLocalSegments(globalMarkers, swerveSequenceSegments));
    }


    // MARKER BUILD UPS
    private List<TrajectoryMarker> convertMarkersToGlobal(
            List<SwerveSequenceSegment> swerveSequenceSegments,
            List<TemporalMarker> temporalMarkers,
            List<DisplacementMarker> displacementMarkers,
            List<SpatialMarker> spatialMarkers
    ) {
        ArrayList<TrajectoryMarker> trajectoryMarkers = new ArrayList<>();

        // convert temporal markers
        for (TemporalMarker marker : temporalMarkers) {
            trajectoryMarkers.add(
                    new TrajectoryMarker(marker.getProducer().produce(currentDuration), marker.getCallback())
            );
        }

        // convert displacement markers
        for (DisplacementMarker marker : displacementMarkers) {
            double time = displacementToTime(
                    swerveSequenceSegments,
                    marker.getProducer().produce(currentDisplacement)
            );

            trajectoryMarkers.add(
                    new TrajectoryMarker(
                            time,
                            marker.getCallback()
                    )
            );
        }

            // convert spatial markers
            for (SpatialMarker marker : spatialMarkers) {
                trajectoryMarkers.add(
                        new TrajectoryMarker(
                                pointToTime(swerveSequenceSegments, marker.getPoint()),
                                marker.getCallback()
                        )
                );
            }

            return trajectoryMarkers;
        }

        private List<SwerveSequenceSegment> projectGlobalMarkersToLocalSegments(List<TrajectoryMarker> markers, List<SwerveSequenceSegment> swerveSequenceSegments) {
            if (swerveSequenceSegments.isEmpty()) return Collections.emptyList();

            double totalSequenceDuration = 0;
            for (SwerveSequenceSegment segment : swerveSequenceSegments) {
                totalSequenceDuration += segment.getDuration();
            }

            for (TrajectoryMarker marker : markers) {
                SwerveSequenceSegment segment = null;
                int segmentIndex = 0;
                double segmentOffsetTime = 0;

                double currentTime = 0;
                for (int i = 0; i < swerveSequenceSegments.size(); i++) {
                    SwerveSequenceSegment seg = swerveSequenceSegments.get(i);

                    double markerTime = Math.min(marker.getTime(), totalSequenceDuration);

                    if (currentTime + seg.getDuration() >= markerTime) {
                        segment = seg;
                        segmentIndex = i;
                        segmentOffsetTime = markerTime - currentTime;

                        break;
                    } else {
                        currentTime += seg.getDuration();
                    }
                }

                SwerveSequenceSegment newSegment = null;

                if (segment instanceof WaitSegment_swerve) {
                    List<TrajectoryMarker> newMarkers = new ArrayList<>(segment.getMarkers());

                    newMarkers.addAll(swerveSequenceSegments.get(segmentIndex).getMarkers());
                    newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                    WaitSegment_swerve thisSegment = (WaitSegment_swerve) segment;
                    newSegment = new WaitSegment_swerve(thisSegment.getStartposeSwerve(), thisSegment.getDuration(), newMarkers);
                } else if (segment instanceof TurnSegment_swerve) {
                    List<TrajectoryMarker> newMarkers = new ArrayList<>(segment.getMarkers());

                    newMarkers.addAll(swerveSequenceSegments.get(segmentIndex).getMarkers());
                    newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                    TurnSegment_swerve thisSegment = (TurnSegment_swerve) segment;
                    newSegment = new TurnSegment_swerve(thisSegment.getStartposeSwerve(), thisSegment.getTotalRotation(), thisSegment.getMotionProfile(), newMarkers);
                } else if (segment instanceof TrajSeg_swerve) {
                    TrajSeg_swerve thisSegment = (TrajSeg_swerve) segment;

                    List<TrajectoryMarker> newMarkers = new ArrayList<>(thisSegment.getTrajectory().getMarkers());
                    newMarkers.add(new TrajectoryMarker(segmentOffsetTime, marker.getCallback()));

                    newSegment = new TrajSeg_swerve(new Trajectory(thisSegment.getTrajectory().getPath(), thisSegment.getTrajectory().getProfile(), newMarkers));

                }

                swerveSequenceSegments.set(segmentIndex, newSegment);
            }
            return swerveSequenceSegments;
        }


    // Taken from Road Runner's TrajectoryGenerator.displacementToTime() since it's private
    // note: this assumes that the profile position is monotonic increasing
    private Double motionProfileDisplacementToTime(MotionProfile profile, double s) {
        double tLo = 0.0;
        double tHi = profile.duration();
        while (!(Math.abs(tLo - tHi) < 1e-6)) {
            double tMid = 0.5 * (tLo + tHi);
            if (profile.get(tMid).getX() > s) {
                tHi = tMid;
            } else {
                tLo = tMid;
            }
        }
        return 0.5 * (tLo + tHi);
    }


    private Double displacementToTime(List<SwerveSequenceSegment> swerveSequenceSegments, double s) {
        double currentTime = 0.0;
        double currentDisplacement = 0.0;

        for (SwerveSequenceSegment segment : swerveSequenceSegments) {
            if (segment instanceof TrajSeg_swerve) {
                TrajSeg_swerve thisSegment = (TrajSeg_swerve) segment;

                double segmentLength = thisSegment.getTrajectory().getPath().length();

                if (currentDisplacement + segmentLength > s) {
                    double target = s - currentDisplacement;
                    double timeInSegment = motionProfileDisplacementToTime(
                            thisSegment.getTrajectory().getProfile(),
                            target
                    );

                    return currentTime + timeInSegment;
                } else {
                    currentDisplacement += segmentLength;
                    currentTime += thisSegment.getTrajectory().duration();
                }
            } else {
                currentTime += segment.getDuration();
            }
        }
        return  0.0;
    }


    private Double pointToTime(List<SwerveSequenceSegment> swerveSequenceSegments, com.acmerobotics.roadrunner.geometry.Vector2d point) {
        class ComparingPoints {
            private final double distanceToPoint;
            private final double totalDisplacement;
            private final double thisPathDisplacement;

            public ComparingPoints(double distanceToPoint, double totalDisplacement, double thisPathDisplacement) {
                this.distanceToPoint = distanceToPoint;
                this.totalDisplacement = totalDisplacement;
                this.thisPathDisplacement = thisPathDisplacement;
            }
        }

        List<ComparingPoints> projectedPoints = new ArrayList<>();

        for (SwerveSequenceSegment segment : swerveSequenceSegments) {
            TrajSeg_swerve thisSegment = (TrajSeg_swerve) segment;

            double displacement = thisSegment.getTrajectory().getPath().project(point, 0.25);
            com.acmerobotics.roadrunner.geometry.Vector2d projectedPoint = thisSegment.getTrajectory().getPath().get(displacement).vec();
            double distanceToPoint = point.minus(projectedPoint).norm();

            double totalDisplacement = 0.0;

            for (ComparingPoints comparingPoint : projectedPoints) {
                totalDisplacement += comparingPoint.totalDisplacement;
            }

            totalDisplacement += displacement;

            projectedPoints.add(new ComparingPoints(distanceToPoint, displacement, totalDisplacement));
        }

    ComparingPoints closestPoint = null;

        for (ComparingPoints comparingPoint : projectedPoints) {
        if (closestPoint == null) {
            closestPoint = comparingPoint;
            continue;
        }

        if (comparingPoint.distanceToPoint < closestPoint.distanceToPoint)
            closestPoint = comparingPoint;
    }

        return displacementToTime(swerveSequenceSegments, closestPoint.thisPathDisplacement);
}


    // CONSTRAINTS
    public TrajSeqBuild_swerve setConstraints(
            TrajectoryVelocityConstraint velConstraint,
            TrajectoryAccelerationConstraint accelConstraint
    ) {
        this.currentVelConstraint = velConstraint;
        this.currentAccelConstraint = accelConstraint;

        return this;
    }

    public TrajSeqBuild_swerve resetConstraints() {
        this.currentVelConstraint = this.baseVelConstraint;
        this.currentAccelConstraint = this.baseAccelConstraint;

        return this;
    }

    public TrajSeqBuild_swerve setVelConstraint(TrajectoryVelocityConstraint velConstraint) {
        this.currentVelConstraint = velConstraint;

        return this;
    }

    public TrajSeqBuild_swerve resetVelConstraint() {
        this.currentVelConstraint = this.baseVelConstraint;

        return this;
    }

    public TrajSeqBuild_swerve setAccelConstraint(TrajectoryAccelerationConstraint accelConstraint) {
        this.currentAccelConstraint = accelConstraint;

        return this;
    }

    public TrajSeqBuild_swerve resetAccelConstraint() {
        this.currentAccelConstraint = this.baseAccelConstraint;

        return this;
    }




    // TRACKING METHODS DO NOT TOUCH
    //TRACKING METHODS
    //methods for path length tracking in autonomous (only useful for driving in straight lines)

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();
    }

    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }

    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }

    private interface AddPathCallBack {
        void run();
    }
}