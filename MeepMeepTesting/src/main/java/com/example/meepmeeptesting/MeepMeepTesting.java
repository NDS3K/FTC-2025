package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Vector2d pose = new Vector2d(-35,35);
        Vector2d pose1 = new Vector2d(-46,12);
        Vector2d pose2 = new Vector2d(-56,12);
        Vector2d pose3 = new Vector2d(-61,12);
        Vector2d pose4 = new Vector2d(-35,50);
        Vector2d pose5 = new Vector2d(-10,40);
        Vector2d pose6 = new Vector2d(-8,40);
        Vector2d pose7 = new Vector2d(-6,40);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-11.5, 60, Math.toRadians(270)))
                .lineToY(38)
                //.splineTo(pose1,Math.toRadians(270))
                .strafeTo(new Vector2d(-20,38))
                .splineToConstantHeading(new Vector2d(-35,25),Math.toRadians(270))
                .splineToConstantHeading(pose1,Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .lineToY(12)
                .strafeTo(pose2)
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .lineToY(12)
                .strafeTo(pose3)
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .strafeTo(pose4)
                .strafeTo(pose5)
                .strafeTo(pose4)
                .strafeTo(pose6)
                .strafeTo(pose4)
                .strafeTo(pose7)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}