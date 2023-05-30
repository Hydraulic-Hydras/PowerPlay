package org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.util.Collections;

public final class TrajSeg_swerve extends SwerveSequenceSegment{

    private final Trajectory trajectory;

    public TrajSeg_swerve(Trajectory trajectory) {
        // markers are stored in the trajectory itself
        // dont hold any markers in this class
        super(trajectory.duration(), trajectory.start(), trajectory.end(), Collections.emptyList());
        this.trajectory = trajectory;

    }

    public Trajectory getTrajectory() { return this.trajectory; }
}
