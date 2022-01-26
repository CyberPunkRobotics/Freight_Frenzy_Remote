package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAgVel, maxAngAccel, track width
                .setConstraints(62.01654253906262, 60, Math.toRadians(140), Math.toRadians(180), 10)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-41.862855949849056, -63.11163142092735, 0))
                        .lineTo(new Vector2d(-60.11550487865194, -58.99897624865089),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                                SampleMecanumDrive.getAccelerationConstraint(15))
                                .splineTo(new Vector2d(-11.638458241408606,-42.3),0)
                                //.strafeLeft(10)
                                .splineToConstantHeading(new Vector2d(27.7,-63.3),0)
                                .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}