package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .setDimensions(18, 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-45 , -60, Math.toRadians(90)))
                //intake off but powered
              //  .strafeTo(new Vector2d(-53, -37))
                //arm is at specimen position
                //.strafeTo(new Vector2d(9, -33))
               // .strafeTo(new Vector2d(9, -43))
               // .setTangent(Math.toRadians(180))
                //.strafeTo(new Vector2d(-34, -34))
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))
                        .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-47, -50, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-58.5, -50, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-60, -50, Math.toRadians(105)), Math.toRadians(105))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-53, -53, Math.toRadians(45)), Math.toRadians(45))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-25, -2, Math.toRadians(0)), Math.toRadians(0))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .waitSeconds(1)


                //.splineToLinearHeading(new Pose2d(-66, -33, Math.toRadians(120)), Math.toRadians(120))
                //.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}