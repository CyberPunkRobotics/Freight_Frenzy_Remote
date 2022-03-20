package org.firstinspires.ftc.teamcode.drive.autonomii_camera;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Disabled
@Autonomous(name="sobranie")
public class Sobranie extends LinearOpMode {

    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d currentPos = null;

        waitForStart();
        if(opModeIsActive()) {



            currentPos = drive.getPoseEstimate();
            TrajectorySequence test = drive.trajectorySequenceBuilder(currentPos)
                        .forward(40,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                                SampleMecanumDrive.getAccelerationConstraint(2))
                        .addTemporalMarker(0, () -> {
                            drive.intake.setPower(0.99);
                            telemetry.update();
                        })
                    .addTemporalMarker(0.5, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(0.6, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive. intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(0.7, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(0.8, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(0.9, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(0.0, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.1, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.2, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.3, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.4, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.5, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.6, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.7, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.8, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(1.9, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.0, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.1, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.2, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.3, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.4, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.5, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.6, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.7, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.8, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(2.9, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })
                    .addTemporalMarker(3.0, () -> { if (drive.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { drive.intake.setPower(0);drive.breakFollowing(); } })

                    .build();

            drive.followTrajectorySequence(test);

            while(drive.distantaIntake.getDistance(DistanceUnit.CM)>1.5 && opModeIsActive())
                drive.intake.setPower(0.99);

            drive.intake.setPower(0);

            drive.updatePoseEstimate();
            currentPos = drive.getPoseEstimate();

            TrajectorySequence test1 = drive.trajectorySequenceBuilder(currentPos)
                    .back(10,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                            SampleMecanumDrive.getAccelerationConstraint(2))
                    .build();
            drive.followTrajectorySequence(test1);

        }
    }
}
