package org.firstinspires.ftc.teamcode.drive.SwerveTrajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SwerveTrajectories.SwervesequenceSegments.SwerveSequenceSegment;

import java.util.Collections;
import java.util.List;

public class TrajSeq_swerve {

    private final List<SwerveSequenceSegment> Sequencelist;

    public TrajSeq_swerve(List<SwerveSequenceSegment> Sequencelist) {
        if (Sequencelist.size() == 0 ) throw new EmptySeqEXP_swerve();

        this.Sequencelist = Collections.unmodifiableList(Sequencelist);
    }

    public Pose2d start() { return Sequencelist.get(0).getStartposeSwerve(); }

    public Pose2d end() { return Sequencelist.get(Sequencelist.size() - 1).getEndPoseSwerve(); }

    public double duration() {
        double total = 0.0;

        for (SwerveSequenceSegment segment : Sequencelist) {
            total += segment.getDuration();
        }
        return total;
    }

    public SwerveSequenceSegment get(int i) { return Sequencelist.get(i); }

    public int size() { return Sequencelist.size(); }
}
