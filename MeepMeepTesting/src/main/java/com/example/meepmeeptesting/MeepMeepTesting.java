package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d collectPos = new Pose2d(28, -60, Math.toRadians(90));
        Pose2d dropoff = new Pose2d(0, -30, Math.toRadians(90));
        Pose2d highbasket = new Pose2d(-56, -56, Math.toRadians(135));
        Vector2d colVec = new Vector2d(34, -40);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        .lineTo(new Vector2d(0, -30))
                        .setReversed(true)
                        .splineTo(colVec, Math.toRadians(270))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/*
                        .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        .lineTo(new Vector2d(0, -30))
                        .lineToLinearHeading(new Pose2d(34, -40, Math.toRadians(45)))
                        .lineToLinearHeading(collectPos)
                        .lineToLinearHeading(dropoff)
                        .lineToLinearHeading(new Pose2d(40, -40, Math.toRadians(45)))
                        .lineToLinearHeading(collectPos)
                        .lineToLinearHeading(dropoff)
                        .lineToLinearHeading(new Pose2d(46, -40, Math.toRadians(45)))
                        .lineToLinearHeading(collectPos)
                        .lineToLinearHeading(dropoff)
                        .lineToLinearHeading(collectPos)
                        .lineToLinearHeading(dropoff)
                        .lineToLinearHeading(new Pose2d(-38, -38, Math.toRadians(130)))
                        .lineToLinearHeading(highbasket)
                        .lineToLinearHeading(new Pose2d(-50, -40, Math.toRadians(120)))
                        .lineToLinearHeading(highbasket)
                        .lineToLinearHeading(new Pose2d(-55, -40, Math.toRadians(130)))
                        .lineToLinearHeading(highbasket)
                        .build());




                        .setConstraints(55, 52.48, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, 62.75, Math.toRadians(-270)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(34, 35.5, Math.toRadians(-180)))
                        .splineToConstantHeading(new Vector2d(54, 43), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(24, 61), Math.toRadians(180))
                        .lineToConstantHeading(new Vector2d(-40, 61))
                        .lineToLinearHeading(new Pose2d(-59.5, 41, Math.toRadians(195)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-35, 61.5, Math.toRadians(180)))
                        .lineToConstantHeading(new Vector2d(12, 61.5))
                        .splineTo(new Vector2d(53, 43), Math.toRadians(0))
                        .setReversed(false)
                        .splineTo(new Vector2d(24, 61), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(-40, 62, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-59.5, 42.5, Math.toRadians(190)))
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, 61), Math.toRadians(0))
                        .lineToConstantHeading(new Vector2d(12, 61))
                        .splineTo(new Vector2d(53, 44), Math.toRadians(0))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(44, 50, Math.toRadians(180)))

 */