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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7 , -61.5, Math.toRadians(90)))
                //intake off but powered

              //  .strafeTo(new Vector2d(-53, -37))
                //arm is at specimen position
                //.strafeTo(new Vector2d(9, -33))
               // .strafeTo(new Vector2d(9, -43))
               // .setTangent(Math.toRadians(180))
                //.strafeTo(new Vector2d(-34, -34))
//
                .splineToLinearHeading(new Pose2d(7, -35, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(35, -43, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(35, -9, Math.toRadians(90)), Math.toRadians(-90))

                .splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(48, -55, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(48, -9, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(57, -9, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(57, -55, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -33, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(57, -9, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(62, -9, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(62, -55, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(40, -57, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(9, -37, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(40, -57, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearHeading(new Pose2d(-58, -50, Math.toRadians(60)), Math.toRadians(60))

                .splineToLinearHeading(new Pose2d(11, -37, Math.toRadians(90)), Math.toRadians(90))//.splineToL

                .splineToLinearHeading(new Pose2d(40, -57, Math.toRadians(90)), Math.toRadians(90))//.splineToLinearH

                .splineToLinearHeading(new Pose2d(13, -37, Math.toRadians(90)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(52, -56, Math.toRadians(90)), Math.toRadians(90))
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}