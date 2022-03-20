package org.firstinspires.ftc.teamcode.drive.autonomii_camera;

import android.media.tv.TvTrackInfo;
import android.net.vcn.VcnConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Autonomous(name="AlbastruPreloadCuburiParcare", group="Auto")
public class AlbastruPreloadCuburiParcare extends LinearOpMode {


    FtcDashboard dashboard;
    OpenCvCamera webcam;
    private SampleMecanumDrive robot = null;
    DetectarePozitie pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(6.142745694204604,64.925208682113));
        robot.setExternalHeading(Math.toRadians(0));

        dashboard = FtcDashboard.getInstance();

        //robot.intake.setPosition(0.35);
        robot.PivotBrat.setPosition(0.5);

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectarePozitie(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                                             dashboard.startCameraStream(webcam, 120);
                                         }

                                         @Override
                                         public void onError(int errorCode) {

                                         }
                                     }

        );


        waitForStart();

        telemetry.addData("unghi", robot.getRawExternalHeading());

        switch (pipeline.getLocation()) {
            case UNU:
                NivelDoi();
                break;
            case DOI:
                NivelTrei();
                break;
            case TREI:
                NivelUnu();
                break;

        }
        webcam.stopStreaming();
    }

    private void NivelUnu() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d currentPose;
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,45.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(49))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(198,1);})
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.47); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.44); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.41); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.35); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.29); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.23); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.20); })
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.17);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.14);})
                .build();

//        sleep(10000);


        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);robot.RidicareBrat(0,0.7);})
                .waitSeconds(0.7)
                .strafeLeft(8)
                .splineToConstantHeading(new Vector2d(38.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(18,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .addTemporalMarker(0.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot. intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
//                .waitSeconds(8)
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, 48.49213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.48); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.46); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.44); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.42); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.40); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.36);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.34); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.30); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.28); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.27); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.25); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.22); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.32);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeLeft(5)
//                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(62))
                .splineToConstantHeading(new Vector2d(34.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .lineToLinearHeading(new Pose2d(50.79158659407203, 60.58922924676717))
                .strafeRight(25)
                .build();

        robot.followTrajectorySequence(mergeInWareHouseDupa1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

    }

    private void NivelDoi() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d currentPose;
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, ()->{robot.RidicareBrat(430,1);})
                .waitSeconds(0.5)
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,46.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(49))
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.47); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.43); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.40); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.36); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.34); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.30); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.28); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.18);})
                .build();

//        sleep(10000);


        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.38);robot.RidicareBrat(0,0.8);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeLeft(7)
                .splineToConstantHeading(new Vector2d(38.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(18,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .addTemporalMarker(0.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot. intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
//                .waitSeconds(8)
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, 48.49213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.48); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.46); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.44); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.42); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.40); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.36);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.34); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.30); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.28); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.24); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.22); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.20); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .turn(0)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.34);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeLeft(6.7)
//                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(62))
                .splineToConstantHeading(new Vector2d(34.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .lineToLinearHeading(new Pose2d(50.79158659407203, 60.58922924676717))
                .strafeRight(25)
                .build();

        robot.followTrajectorySequence(mergeInWareHouseDupa1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

    }

    private void NivelTrei() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d currentPose;
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence punePreload = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, ()->{robot.RidicareBrat(730,1);})
                .waitSeconds(0.65)
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,46.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.47); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.44); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.42); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.40); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.36); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.34); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.24); })
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.16);})
                .build();

//        sleep(10000);

        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.40);robot.RidicareBrat(0,0.8);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.49);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeLeft(7)
                .splineToConstantHeading(new Vector2d(38.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(18,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .addTemporalMarker(0.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot. intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(0.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(1.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(2.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.3, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.4, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.5, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.6, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.7, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.8, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(3.9, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.1, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .addTemporalMarker(4.2, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
//                .waitSeconds(8)
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, 48.49213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.48); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.46); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.44); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.42); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.40); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.38); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.36);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.34); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.32); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.30); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.28); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.26); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.24); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.22); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.21); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .turn(0)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.23);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.25);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.27);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.29);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.31);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.32);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeLeft(6.7)
//                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(62))
                .splineToConstantHeading(new Vector2d(34.79158659407203, 62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .lineToLinearHeading(new Pose2d(50.79158659407203, 60.58922924676717))
                .strafeRight(25)
                .build();

        robot.followTrajectorySequence(mergeInWareHouseDupa1);

        robot.updatePoseEstimate();

        currentPose = robot.getPoseEstimate();

    }



    private void stopDriving(){
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setPower(0);
        robot.getLastTick(robot.ridicareBrat.getCurrentPosition());
    }




}
