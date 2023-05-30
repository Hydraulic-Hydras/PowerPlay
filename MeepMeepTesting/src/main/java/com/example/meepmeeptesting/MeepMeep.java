package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep {
    public static void main(String[] args) {
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(360), Math.toRadians(360), 15)
                .setDimensions(15,17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -64, Math.toRadians(90)))


                                // RED RIGHT SIDE
                                // pre load
                                .splineTo(new Vector2d(36.67, -30.50), Math.toRadians(106.73))
                                .splineTo(new Vector2d(31.17, -7.17), Math.toRadians(136.17))
                                .waitSeconds(1)

                                // one cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                                .waitSeconds(1)


                                // two cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                                .waitSeconds(1)


                                // three cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                                .waitSeconds(1)


                                // four cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                                .waitSeconds(1)


                                // five cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(58.50, -12.30), Math.toRadians(-2.16))
                                .waitSeconds(1)

                                .setReversed(false)
                                .splineTo(new Vector2d(30.00, -8.33), Math.toRadians(128.66))
                                .waitSeconds(1)


                                .build()
                );

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}