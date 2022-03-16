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

@Autonomous(name="PreloadCuburiParcare", group="Auto")
public class PreloadCuburiParcare extends LinearOpMode {


    FtcDashboard dashboard;
    OpenCvCamera webcam;
    private SampleMecanumDrive robot = null;
    DetectarePozitie pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(6.142745694204604,-64.925208682113));
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
                NivelUnu();
                break;
            case DOI:
                NivelDoi();
                break;
            case TREI:
                NivelTrei();
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
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,-46.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(49))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(198,1);})
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.8); })
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.83);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.86);})
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.84);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.82);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.80);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.78);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.66);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.62);robot.RidicareBrat(0,0.8);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(7)
                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(10,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
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
                .addTemporalMarker(3.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,-65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, -49.89213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.64);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.75); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.77); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.66);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(4.5)
                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
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
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,-46.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(49))
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.8); })
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.83);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.86);})
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.84);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.82);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.80);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.78);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.66);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.62);robot.RidicareBrat(0,0.8);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(7)
                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(10,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
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
                .addTemporalMarker(3.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,-65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, -49.89213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.64);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.75); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.77); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .turn(0)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.66);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(6.7)
                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
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
                .lineToConstantHeading(new Vector2d(-0.6619354985385653,-46.34777575882209),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.8); })
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.83);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.86);})
                .build();

        robot.followTrajectorySequence(punePreload);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0  ,()->{robot.intake.setPower(-0.99);}) //scuipa cubu
                .addTemporalMarker(0.6,()->{robot.intake.setPower(0);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.84);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.82);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.80);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.78);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.66);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.62);robot.RidicareBrat(0,0.8);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(7)
                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
                .build();

        robot.followTrajectorySequence(mergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence iaCub = robot.trajectorySequenceBuilder(currentPose)
                .forward(10,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
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
                .addTemporalMarker(3.0, () -> { if (robot.distantaIntake.getDistance(DistanceUnit.CM) <= 2.5) { robot.intake.setPower(0);robot.breakFollowing(); } })
                .build();

        robot.followTrajectorySequence(iaCub);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCub1 = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, () -> { robot.intake.setPower(0.99);})
                .lineTo(new Vector2d(28.004453496583984,-65.56759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-3.100778017390152, -49.89213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(740,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.64);robot.intake.setPower(0); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.75); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.77); })
                .build();

        robot.followTrajectorySequence(PuneCub1);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence mergeInWareHouseDupa1 = robot.trajectorySequenceBuilder(currentPose)
                .turn(0)
                .addTemporalMarker(0  , ()->{ robot.intake.setPower(-0.99);})
                .addTemporalMarker(0.6  ,()->{robot.RidicareBrat(0,1);})
                .addTemporalMarker(0.6,()->{robot.PivotBrat.setPosition(0.76);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.74);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.72);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.70);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.66);robot.intake.setPower(0);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.60);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.56);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.52);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.50);})
                .waitSeconds(0.7)
                .strafeRight(6.7)
                .splineToConstantHeading(new Vector2d(42.79158659407203, -62.58922924676717),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(62))
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
