package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double rad(double angleDeg){
        return Math.toRadians(angleDeg);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 9.28)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -65, Math.toRadians(-90)))
                        .back(32)
                        .waitSeconds(1)
//                        .lineTo(new Vector2d(0,35))
//                        .turn(rad(90))
                        .splineTo(new Vector2d(5,-43), Math.toRadians(0))
                        .splineTo(new Vector2d(58, -40), Math.toRadians(90))
//                        .forward(5)
//                        .splineTo(new Vector2d(38, -20), Math.toRadians(90))
//                        .splineTo(new Vector2d(46, -6), Math.toRadians(90)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}