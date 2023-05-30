package org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.Angle;

import java.util.List;

public class TurnSegment_swerve extends SwerveSequenceSegment{

    private final double totalRotation;

    private final MotionProfile motionProfile;

    public TurnSegment_swerve(Pose2d startpose, double totalRotation, MotionProfile motionProfile, List<TrajectoryMarker> markers) {
        super(
                motionProfile.duration(),
                startpose,
                new Pose2d(
                        startpose.getX(), startpose.getY(),
                        Angle.norm(startpose.getHeading() + totalRotation)
                ),
                markers
        );

        this.totalRotation = totalRotation;
        this.motionProfile = motionProfile;
    }

    public final double getTotalRotation() { return this.totalRotation; }

    public final MotionProfile getMotionProfile() { return  this.motionProfile; }
}
