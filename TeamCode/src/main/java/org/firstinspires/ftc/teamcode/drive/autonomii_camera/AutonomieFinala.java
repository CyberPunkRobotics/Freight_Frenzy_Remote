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
    private ElapsedTime time;
    private LinearOpMode opMode;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(-41.8157286018119,-64.5442651728134));
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


            TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-60.11550487865194, -58.99897624865089, Math.toRadians(28)),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                            SampleMecanumDrive.getAccelerationConstraint(15))
                    .addTemporalMarker(1, ()->{robot.rata.setPower(1);})
                    .waitSeconds(2)
                    .addTemporalMarker(3.2,()->{robot.rata.setPower(0);})
                    .build();

            robot.followTrajectorySequence(RotireRata);
//323
            robot.updatePoseEstimate();
            currentPose = robot.getPoseEstimate();
            TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                    .splineToLinearHeading(new Pose2d( -30.46965582232959, -42.95435473987366, Math.toRadians(323)),0)
                    .addTemporalMarker(0, ()->{robot.RidicareBrat(675,1); })
                    .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.53); })
                    .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.56); })
                    .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.59); })
                    .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                    .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                    .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.68); })
                    .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.71); })
                    .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.74); })
                    .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.76); })
                    .addTemporalMarker(1.8, ()->{robot.intake.setPower(-0.99);})
                    .waitSeconds(0.5)
                    .addTemporalMarker(2.3, ()->{ robot.intake.setPower(0);robot.PivotBrat.setPosition(0.74); })
                    .addTemporalMarker(2.4, ()->{ robot.PivotBrat.setPosition(0.73); })
                    .addTemporalMarker(2.5, ()->{ robot.PivotBrat.setPosition(0.71); })
                    .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.69); })
                    .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.67); })
                    .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.65); })
                    .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.63); })
                    .addTemporalMarker(3.0, ()->{ robot.PivotBrat.setPosition(0.61); })
                    .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.59); })
                    .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.57); })
                    .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.55); })
                    .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.53); })
                    .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.5); })
                    .addTemporalMarker(2.7,()->{robot.RidicareBrat(0,1);})
                    .splineToSplineHeading(new Pose2d(33.79158659407203, -62.58922924676717, Math.toRadians(0)),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                            SampleMecanumDrive.getAccelerationConstraint(50))
                    .build();

            robot.followTrajectorySequence(PuneCubPeNivel);

        time = new ElapsedTime();

        while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 3.6 && time.time()< 5 ){
            robot.updatePoseEstimate();
            currentPose = robot.getPoseEstimate();
            robot.leftFront.setPower(0.05);
            robot.leftRear.setPower(0.05);
            robot.rightFront.setPower(0.05);
            robot.rightRear.setPower(0.05);
            robot.intake.setPower(0.99);
        }
        time.reset();

        stopDriving();
        while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.4){
            robot.updatePoseEstimate();
            robot.intake.setPower(0.99);
        }
        stopDriving();

        robot.updatePoseEstimate();
        currentPose= robot.getPoseEstimate();

        TrajectorySequence mergeInWh = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(28.004453496583984,-63.46759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .splineToConstantHeading(new Vector2d(-10.800778017390152, -46.49213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(720,0.7);})
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.64); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.75); })
                .waitSeconds(1.5)
                .addTemporalMarker(3.7, ()->{ robot.intake.setPower(-0.99);})
                .waitSeconds(0.5)
                .strafeRight(5.8)
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4.6, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.7, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.8, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.9, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(5.0, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(5.1, ()->{ robot.PivotBrat.setPosition(0.50); })
                .addTemporalMarker(5.5, ()->{robot.RidicareBrat(0,1);})
                .splineToConstantHeading(new Vector2d(37.79158659407203, -62.58922924676717),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        robot.followTrajectorySequence(mergeInWh);


    }


    private void stopDriving(){
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setPower(0);
    }


}
