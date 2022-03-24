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

@Autonomous(name="RosuRataPreloadParcare", group="Auto")
public class RosuRataPreloadParcare extends LinearOpMode {


    FtcDashboard dashboard;
    OpenCvCamera webcam;
    private SampleMecanumDrive robot = null;
    DetectarePozitie pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(-41.8157286018119,-64.5442651728134));
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

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(220,0.2);})
                .lineTo(new Vector2d(-62.91550487865194, -61.5),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(1, ()->{robot.rata.setPower(0.55);}) //0.55 0.6
                .waitSeconds(2.3)
                .addTemporalMarker(3.3,()->{robot.rata.setPower(0);})
//                .forward(10)
//                .strafeRight(4)
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(()->{robot.rata.setPower(0);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -41.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(220,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(2.4, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65);robot.intake.setPower(0); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(4,()->{robot.RidicareBrat(0,0.4);})
                .waitSeconds(0.1)
                .addTemporalMarker(3.7,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(3.8,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(3.9,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(4,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(4.1,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(4.2,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(4.3,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(4.4,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(4.5,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(4.6,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(4.7,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(4.8,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(4.9,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(5,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(5.1,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(5.2,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(5.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(5.4,()->{robot.PivotBrat.setPosition(0.14);})
                .addTemporalMarker(5.5,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(5.6,()->{robot.PivotBrat.setPosition(0.10);})
                .addTemporalMarker(5.7,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(5.8,()->{robot.PivotBrat.setPosition(0.06);})
                .addTemporalMarker(5.9,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(6,()->{robot.PivotBrat.setPosition(0.02);})
                .addTemporalMarker(6.1,()->{robot.PivotBrat.setPosition(0.01);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -68.0442651728134),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu !!!nu se sterge

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence IaRata = robot.trajectorySequenceBuilder(currentPose)
                .back(22,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(5))
                .addTemporalMarker(0,()->{robot.intake.setPower(0.99);})
                .build();

        robot.followTrajectorySequence(IaRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(  -25.38146174877701, -46.56293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(0.6,()->{robot.RidicareBrat(725,1);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.12);robot.intake.setPower(0);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.65);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.5)
                .build();

        robot.followTrajectorySequence(PuneRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        robot.RidicareBrat(0,0.8);

        TrajectorySequence Parcheaza = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(2)
                .splineToConstantHeading(new Vector2d(-63.505756564194397,-41.00075310738899),170,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .back(2)
                .addTemporalMarker(()->{robot.intake.setPower(0);})
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.63);robot.intake.setPower(0); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);})
                .build();

        robot.followTrajectorySequence(Parcheaza);




       /* TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(750,1);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -48.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(1,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.4,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);robot.RidicareBrat(0,1);})
                .waitSeconds(0.5)
                .forward(75,SampleMecanumDrive.getVelocityConstraint(30, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        robot.followTrajectorySequence(puneRata);*/





    }

    private void NivelDoi() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(480,0.2);})
                .lineTo(new Vector2d(-62.91550487865194, -61.5),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(1, ()->{robot.rata.setPower(0.55);}) //0.55 0.6
                .waitSeconds(2.3)
                .addTemporalMarker(3.3,()->{robot.rata.setPower(0);})
//                .forward(10)
//                .strafeRight(4)
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(()->{robot.rata.setPower(0);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -44.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(480,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(2.4, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65);robot.intake.setPower(0); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(3.3,()->{robot.RidicareBrat(0,0.4);})
                .waitSeconds(0.1)
                .addTemporalMarker(3.7,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(3.8,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(3.9,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(4,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(4.1,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(4.2,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(4.3,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(4.4,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(4.5,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(4.6,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(4.7,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(4.8,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(4.9,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(5,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(5.1,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(5.2,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(5.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(5.4,()->{robot.PivotBrat.setPosition(0.14);})
                .addTemporalMarker(5.5,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(5.6,()->{robot.PivotBrat.setPosition(0.10);})
                .addTemporalMarker(5.7,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(5.8,()->{robot.PivotBrat.setPosition(0.06);})
                .addTemporalMarker(5.9,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(6,()->{robot.PivotBrat.setPosition(0.02);})
                .addTemporalMarker(6.1,()->{robot.PivotBrat.setPosition(0.01);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -68.0442651728134),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu !!!nu se sterge

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence IaRata = robot.trajectorySequenceBuilder(currentPose)
                .back(22,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(5))
                .addTemporalMarker(0,()->{robot.intake.setPower(0.99);})
                .build();

        robot.followTrajectorySequence(IaRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(-25.38146174877701, -46.56293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(0.6,()->{robot.RidicareBrat(725,1);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.12);robot.intake.setPower(0);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.65);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.5)
                .build();

        robot.followTrajectorySequence(PuneRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        robot.RidicareBrat(0,0.8);

        TrajectorySequence Parcheaza = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(2)
                .splineToConstantHeading(new Vector2d(-63.505756564194397,-41.00075310738899),170,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .back(2)
                .addTemporalMarker(()->{robot.intake.setPower(0);})
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.63);robot.intake.setPower(0); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);})
                .build();

        robot.followTrajectorySequence(Parcheaza);




       /* TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(750,1);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -48.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(1,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.4,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);robot.RidicareBrat(0,1);})
                .waitSeconds(0.5)
                .forward(75,SampleMecanumDrive.getVelocityConstraint(30, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        robot.followTrajectorySequence(puneRata);*/



    }

    private void NivelTrei() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(700,0.2);})
                .lineTo(new Vector2d(-62.91550487865194, -61.5),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(1, ()->{robot.rata.setPower(0.55);}) //0.55 0.6
                .waitSeconds(2.3)
                .addTemporalMarker(3.3,()->{robot.rata.setPower(0);})
//                .forward(10)
//                .strafeRight(4)
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(()->{robot.rata.setPower(0);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -44.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(750,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(2.4, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65);robot.intake.setPower(0); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(3.3,()->{robot.RidicareBrat(0,0.4);})
                .waitSeconds(0.1)
                .addTemporalMarker(3.7,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(3.8,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(3.9,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(4,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(4.1,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(4.2,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(4.3,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(4.4,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(4.5,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(4.6,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(4.7,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(4.8,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(4.9,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(5,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(5.1,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(5.2,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(5.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(5.4,()->{robot.PivotBrat.setPosition(0.14);})
                .addTemporalMarker(5.5,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(5.6,()->{robot.PivotBrat.setPosition(0.10);})
                .addTemporalMarker(5.7,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(5.8,()->{robot.PivotBrat.setPosition(0.06);})
                .addTemporalMarker(5.9,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(6,()->{robot.PivotBrat.setPosition(0.02);})
                .addTemporalMarker(6.1,()->{robot.PivotBrat.setPosition(0.01);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -68.0442651728134),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(2))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu !!!nu se sterge

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence IaRata = robot.trajectorySequenceBuilder(currentPose)
                .back(22,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(5))
                .addTemporalMarker(0,()->{robot.intake.setPower(0.99);})
                .build();

        robot.followTrajectorySequence(IaRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(  -24.38146174877701, -46.56293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(0.6,()->{robot.RidicareBrat(725,1);})
                .addTemporalMarker(0.7,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(0.8,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(0.9,()->{robot.PivotBrat.setPosition(0.12);robot.intake.setPower(0);})
                .addTemporalMarker(1.0,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.65);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.68);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.5)
                .build();

        robot.followTrajectorySequence(PuneRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        robot.RidicareBrat(0,0.8);

        TrajectorySequence Parcheaza = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(2)
                .splineToConstantHeading(new Vector2d(-63.505756564194397,-41.00075310738899),170,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .back(2)
                .addTemporalMarker(()->{robot.intake.setPower(0);})
                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.63);robot.intake.setPower(0); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);})
                .build();

        robot.followTrajectorySequence(Parcheaza);




       /* TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0,()->{robot.RidicareBrat(750,1);})
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -48.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .addTemporalMarker(1,()->{robot.PivotBrat.setPosition(0.04);})
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.08);})
                .addTemporalMarker(1.2,()->{robot.PivotBrat.setPosition(0.12);})
                .addTemporalMarker(1.3,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(1.4,()->{robot.PivotBrat.setPosition(0.2);})
                .addTemporalMarker(1.5,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.4);})
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.54);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.58);})
                .addTemporalMarker(2.4,()->{robot.PivotBrat.setPosition(0.62);})
                .addTemporalMarker(2.5,()->{robot.PivotBrat.setPosition(0.64);})
                .addTemporalMarker(2.6,()->{robot.intake.setPower(-0.99);})
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0);robot.RidicareBrat(0,1);})
                .waitSeconds(0.5)
                .forward(75,SampleMecanumDrive.getVelocityConstraint(30, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .build();

        robot.followTrajectorySequence(puneRata);*/


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
