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
public class RosuRataParcare extends LinearOpMode {


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

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -43.96293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(220,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.66); })
//                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(1.3, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
//                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
//                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
//                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
//                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(2.1, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(2.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(2.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(2.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(2.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0); })
                .addTemporalMarker(2.5,()->{robot.RidicareBrat(0,1);})
                .lineToConstantHeading(new Vector2d(-62.91550487865194, -61.89897624865089),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

        robot.followTrajectorySequence(PuneCubPeNivel);


        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, ()->{robot.rata.setPower(0.65);})
                .waitSeconds(2)
                .addTemporalMarker(2.1,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();



    }

    private void NivelDoi() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -43.96293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(480,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.66); })
//                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(2, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
//                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
//                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
//                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
//                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.0, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0); })
                .addTemporalMarker(3.5,()->{robot.RidicareBrat(0,1);})
                .lineToConstantHeading(new Vector2d(-62.91550487865194, -61.89897624865089),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

        robot.followTrajectorySequence(PuneCubPeNivel);


        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, ()->{robot.rata.setPower(0.65);})
                .waitSeconds(2)
                .addTemporalMarker(2.1,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();


    }

    private void NivelTrei() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();

        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(  -27.98146174877701, -44.06293767005846),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0, ()->{robot.RidicareBrat(750,1); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(1.3, ()->{ robot.PivotBrat.setPosition(0.66); })
//                .addTemporalMarker(1.1, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(2.4, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.7)
//                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
//                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
//                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
//                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5);robot.intake.setPower(0); })
                .addTemporalMarker(3.3,()->{robot.RidicareBrat(0,1);})
                .waitSeconds(0.1)
                .lineToConstantHeading(new Vector2d(-62.91550487865194, -62.39897624865089),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build(); //daca nu merge cu nationala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

        robot.followTrajectorySequence(PuneCubPeNivel);


        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .addTemporalMarker(0, ()->{robot.rata.setPower(0.65);})
                .waitSeconds(2)
                .addTemporalMarker(2.1,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence IaRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToConstantHeading(new Vector2d(-33.468215750568035,-51.04043764921476))
                .addTemporalMarker( ()->robot.RidicareBrat(210,1))
                .waitSeconds(4)
                .addTemporalMarker(1.5  ,()->{robot.PivotBrat.setPosition(0.48);})
                .addTemporalMarker(1.6,()->{robot.PivotBrat.setPosition(0.46);})
                .addTemporalMarker(1.7,()->{robot.PivotBrat.setPosition(0.44);})
                .addTemporalMarker(1.8,()->{robot.PivotBrat.setPosition(0.42);})
                .addTemporalMarker(1.9,()->{robot.PivotBrat.setPosition(0.40);})
                .addTemporalMarker(2.0,()->{robot.PivotBrat.setPosition(0.38);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.36);})
                .addTemporalMarker(2.2,()->{robot.PivotBrat.setPosition(0.34);})
                .addTemporalMarker(2.3,()->{robot.PivotBrat.setPosition(0.32);})
                .addTemporalMarker(2.4,()->{robot.PivotBrat.setPosition(0.30);})
                .addTemporalMarker(2.5,()->{robot.PivotBrat.setPosition(0.28);})
                .addTemporalMarker(2.6,()->{robot.PivotBrat.setPosition(0.26);})
                .addTemporalMarker(2.7,()->{robot.PivotBrat.setPosition(0.24);})
                .addTemporalMarker(2.8,()->{robot.PivotBrat.setPosition(0.22);})
                .addTemporalMarker(2.9,()->{robot.PivotBrat.setPosition(0.20);})
                .addTemporalMarker(3.0,()->{robot.PivotBrat.setPosition(0.18);})
                .addTemporalMarker(3.1,()->{robot.PivotBrat.setPosition(0.16);})
                .addTemporalMarker(3.2,()->{robot.PivotBrat.setPosition(0.14);})
                .addTemporalMarker(3.3,()->{robot.PivotBrat.setPosition(0.12);})
                .build();

        robot.followTrajectorySequence(IaRata);



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
