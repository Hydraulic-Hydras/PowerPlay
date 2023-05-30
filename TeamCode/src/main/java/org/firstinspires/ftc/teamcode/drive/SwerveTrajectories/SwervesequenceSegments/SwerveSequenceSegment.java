package org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;

public abstract class SwerveSequenceSegment {

    private final double duration;

    private final Pose2d startpose;

    private final Pose2d endPose;

    private final List<TrajectoryMarker> markers;

    protected SwerveSequenceSegment(
            double duration,
            Pose2d startpose, Pose2d endPose,
            List<TrajectoryMarker> markers
    ) {
        this.duration = duration;
        this.startpose = startpose;
        this.endPose = endPose;
        this.markers = markers;
    }

    public double getDuration() { return this.duration; }

    public Pose2d getStartposeSwerve() { return this.startpose; }

    public Pose2d getEndPoseSwerve() { return this.endPose; }

    public List<TrajectoryMarker> getMarkers() { return markers; }

}
