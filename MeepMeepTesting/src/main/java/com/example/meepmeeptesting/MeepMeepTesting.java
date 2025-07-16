package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13, 13)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30.5, -65, Math.toRadians(90)))
                        //Middle Spec Auto
                        .strafeToConstantHeading(new Vector2d(-12, -25))
                        .strafeToConstantHeading(new Vector2d(-12, -6))
                        .strafeToConstantHeading(new Vector2d(-20, -6))
                        .strafeToConstantHeading(new Vector2d(-12, -6))

                        .strafeToConstantHeading(new Vector2d(-12, -32))
                        .strafeToConstantHeading(new Vector2d(-7, -32))
                        .strafeToConstantHeading(new Vector2d(-12, -65))

                        .strafeToConstantHeading(new Vector2d(-12, -8))
                        .strafeToConstantHeading(new Vector2d(-20, -8))
                        .strafeToConstantHeading(new Vector2d(-12, -8))

                        .strafeToConstantHeading(new Vector2d(-12, -22))
                        .strafeToConstantHeading(new Vector2d(-7, -22))
                        .strafeToConstantHeading(new Vector2d(-12, -65))

                        .strafeToConstantHeading(new Vector2d(-12, -10))
                        .strafeToConstantHeading(new Vector2d(-20, -10))

                        .strafeToConstantHeading(new Vector2d(-7, -12))
                        .strafeToConstantHeading(new Vector2d(-12, -65))



                        .strafeToConstantHeading(new Vector2d(-12, -12))
                        .strafeToConstantHeading(new Vector2d(-20, -12))
                        .strafeToConstantHeading(new Vector2d(-12, -12))
                        .strafeToConstantHeading(new Vector2d(-12, -65))

                .strafeToConstantHeading(new Vector2d(-12, -14))
                .strafeToConstantHeading(new Vector2d(-20, -14))
                .strafeToConstantHeading(new Vector2d(-12, -14))
                .strafeToConstantHeading(new Vector2d(-12, -65))

                .strafeToConstantHeading(new Vector2d(-12, -16))
                .strafeToConstantHeading(new Vector2d(-20, -16))
                .strafeToConstantHeading(new Vector2d(-12, -16))
                .strafeToConstantHeading(new Vector2d(-12, -65))

                .strafeToConstantHeading(new Vector2d(-12, -18))
                .strafeToConstantHeading(new Vector2d(-20, -18))
                .strafeToConstantHeading(new Vector2d(-12, -18))
                .strafeToConstantHeading(new Vector2d(-12, -65))



                        //red sample auto with turning
//                        .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(45))
//                        .strafeToLinearHeading(new Vector2d(-60, -50), Math.toRadians(63))
//                        .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(45))
//                        .strafeToLinearHeading(new Vector2d(-58.5, -50), Math.toRadians(90))
//                        .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(45))
//                        .strafeToLinearHeading(new Vector2d(-64, -50), Math.toRadians(90))
//                        .strafeToLinearHeading(new Vector2d(-60, -60), Math.toRadians(45))
//                        .strafeToLinearHeading(new Vector2d(-13, -35), Math.toRadians(270))
//                        .strafeToConstantHeading(new Vector2d(-13, -25))

                        //.strafeToLinearHeading(new Vector2d(0,-56), Math.toRadians(0))
                        .build());
//
        BufferedImage img = null;
//        try { img = ImageIO.read(new File("MeepMeepTesting/CRIRedSample.png")); }
//        try { img = ImageIO.read(new File("MeepMeepTesting/CRIBlueSample.png")); }
        try { img = ImageIO.read(new File("MeepMeepTesting/CRISpec.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}