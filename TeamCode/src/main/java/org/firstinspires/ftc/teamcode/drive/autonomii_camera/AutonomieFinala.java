package org.firstinspires.ftc.teamcode.drive.autonomii_camera;

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

@Autonomous(name="Autonomie", group="Auto")
public class AutonomieFinala extends LinearOpMode {


    FtcDashboard dashboard;
    OpenCvCamera webcam;
    private SampleMecanumDrive robot = null;
    DetectarePozitie pipeline;
    double dI;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(-41.862855949849056, -63.11163142092735));
        robot.setExternalHeading(Math.toRadians(0));

        dashboard = FtcDashboard.getInstance();

        //robot.intake.setPosition(0.35);
        robot.PivotBrat.setPosition(0.5);

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
        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(-60.11550487865194, -58.99897624865089), SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .addTemporalMarker(1, () -> {
                    robot.rata.setPower(1);
                })
                .waitSeconds(2)
                .addTemporalMarker(3.2, () -> {
                    robot.rata.setPower(0);
                })
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .splineTo(new Vector2d(-16.500778017390152, -45.244871629663166), 0)
                .addTemporalMarker(0, () -> {
                    robot.RidicareBrat(119, 1);
                })
                .addDisplacementMarker(43, () -> {
                    robot.PivotBrat.setPosition(0.77);
                })
                //.addTemporalMarker(9.2, ()->{robot.intake.setPosition(0.2);})
                .waitSeconds(0.5)
                .build();

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence MergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(5.3, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(36))
                .addTemporalMarker(0.5, () -> {
                    robot.PivotBrat.setPosition(0.65);
                })
                .addTemporalMarker(1, () -> {
                    robot.RidicareBrat(0, 0.5);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.PivotBrat.setPosition(0.5);
                })
                .splineToConstantHeading(new Vector2d(35.741806061837597, -63.35317556426468), 0, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        robot.followTrajectorySequence(MergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

    }

    private void NivelDoi() {
        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(-60.11550487865194, -58.99897624865089), SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .addTemporalMarker(1, () -> {
                    robot.rata.setPower(1);
                })
                .waitSeconds(2)
                .addTemporalMarker(3.2, () -> {
                    robot.rata.setPower(0);
                })
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .splineTo(new Vector2d(-16.500778017390152, -45.67213093601243), 0)
                .addTemporalMarker(0, () -> {
                    robot.RidicareBrat(276, 1);
                })
                .addDisplacementMarker(43, () -> {
                    robot.PivotBrat.setPosition(0.77);
                })
                //.addTemporalMarker(7.5, ()->{robot.intake.setPosition(0.2);})
                .waitSeconds(0.5)
                .build();

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence MergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(5.3, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(36))
                .addTemporalMarker(0.5, () -> {
                    robot.PivotBrat.setPosition(0.5);
                })
                .addTemporalMarker(0.4, () -> {
                    robot.RidicareBrat(0, 0.5);
                })
                .splineToConstantHeading(new Vector2d(35.741806061837597, -63.35317556426468), 0, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414, 10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        robot.followTrajectorySequence(MergeInWarehouse);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

    }

    private void NivelTrei() {
        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        dI = robot.distantaIntake.getDistance(DistanceUnit.CM);

            TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
//                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.53); })
//                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.56); })
//                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.59); })
//                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.62); })
//                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.65); })
//                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.68); })
//                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.71); })
//                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.74); })
//                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.76); })
                    .forward(15, SampleMecanumDrive.getVelocityConstraint(5, 5.788888931274414, 10),
                            SampleMecanumDrive.getAccelerationConstraint(2))
                    .addDisplacementMarker(1, () -> { robot.intake.setPower(0.99); })
                    .strafeRight(4)
                    .strafeLeft(4)
                    .build();
            robot.followTrajectorySequence(RotireRata);
    }
}
