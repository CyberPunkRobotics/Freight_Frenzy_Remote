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

import java.sql.Time;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomie", group="Auto")
public class AutonomieFinala extends LinearOpMode {


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
                .lineToLinearHeading(new Pose2d(-60.11550487865194, -58.99897624865089, Math.toRadians(28)),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(1.5,()-> {robot.RidicareBrat(150,1);})
                .addTemporalMarker(1, ()->{robot.rata.setPower(0.6);})
                .waitSeconds(2)
                .addTemporalMarker(3.2,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(  -25.68310486038491, -45.50910491865442, Math.toRadians(330)),0)
                .addTemporalMarker(0, ()->{robot.RidicareBrat(241,1); })
                .addTemporalMarker(0.2, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.3, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.76); })
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.78);})
                .addTemporalMarker(1.7, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.8)
                .addTemporalMarker(2.5, ()->{ robot.intake.setPower(0);robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                .addTemporalMarker(3.0, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(3.2,()->{robot.RidicareBrat(0,1);})
                .splineToSplineHeading(new Pose2d(31.79158659407203, -62.58922924676717, Math.toRadians(0)),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build(); //daca nu merge cu regionala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

            /*
            heading (deg): 53.722214294047895
            headingError (deg): 0.0
            x: -1.9708310002547287
            xError: 0.0
            y: -43.531214475866946
    yError: 0.0
             */
        robot.followTrajectorySequence(PuneCubPeNivel);

//        if(cub_in_raza){
//            runtime.reset();
//            //incercam sa luam cubul, dar daca in 2s nu o luat cubul ne oprim
//            while(robot.distantaIntake.getDistance(DistanceUnit.CM) >=1.3 && runtime.time()<2 && opModeIsActive() )
//                robot.intake.setPower(0.99);
//            stopDriving();
//            if(runtime.time() >= 2 && robot.distantaIntake.getDistance(DistanceUnit.CM) >= 1.3){
//                runtime.reset();
//                //dam robotul in spate
//                while(runtime.time() < 0.9 && opModeIsActive() ) {
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(-0.05);
//                    robot.rightFront.setPower(-0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//                runtime.reset();
//                //dam robotul in stanga
//                while(runtime.time()< 0.7 && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//
//                runtime.reset();
//                //mergem iar in fata si incercam sa luam cubul
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 4 && runtime.time()< 3.5  && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(0.05);
//                    robot.intake.setPower(0.99);
//                }
//                stopDriving();
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
//                    robot.intake.setPower(0.99);
//            }
//        }

        boolean cub_in_raza = false;
        //merge in fata pana vede ceva

        //      runtime.reset();
//        while (runtime.seconds() < 5) {
//            telemetry.addData("Distanta ", robot.distantaIntake.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
        runtime.reset();
        double power = 0.1;
        while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && runtime.time()<5 && opModeIsActive()){
            robot.updatePoseEstimate();
            telemetry.addData("Timp ", runtime.time());
            telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(robot.distantaIntake.getDistance(DistanceUnit.CM) < 3.4)
                power = 0;
        }
        stopDriving();

//        if(cub_in_raza){
//            runtime.reset();
//            //incercam sa luam cubul, dar daca in 2s nu o luat cubul ne oprim
//            while(robot.distantaIntake.getDistance(DistanceUnit.CM) >=1.3 && runtime.time()<2 && opModeIsActive() )
//                robot.intake.setPower(0.99);
//            stopDriving();
//            if(runtime.time() >= 2 && robot.distantaIntake.getDistance(DistanceUnit.CM) >= 1.3){
//                runtime.reset();
//                //dam robotul in spate
//                while(runtime.time() < 0.9 && opModeIsActive() ) {
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(-0.05);
//                    robot.rightFront.setPower(-0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//                runtime.reset();
//                //dam robotul in stanga
//                while(runtime.time()< 0.7 && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//
//                runtime.reset();
//                //mergem iar in fata si incercam sa luam cubul
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 3.4 && runtime.time()< 3.5  && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(0.05);
//                    robot.intake.setPower(0.99);
//                }
//                stopDriving();
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
//                    robot.intake.setPower(0.99);
//            }
//        }

        //while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
        //robot.intake.setPower(0.99);

        robot.updatePoseEstimate();
        currentPose= robot.getPoseEstimate();

        TrajectorySequence mergeInWh = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(28.004453496583984,-63.46759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-10.800778017390152, -48.49213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(720,0.7);robot.intake.setPower(0);})
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
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.77);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.79);})
                .waitSeconds(0.5)
                .addTemporalMarker(3.2, ()->{ robot.intake.setPower(-0.99);})
                .waitSeconds(0.2)
                .addTemporalMarker(4,()->{robot.intake.setPower(0);})
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(3.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.4, ()->{robot.RidicareBrat(0,1);})
                .waitSeconds(0.7)
//                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(-1.9708310002547287, -48.931214475866946,Math.toRadians(70.6)))
                .build();

        robot.followTrajectorySequence(mergeInWh);

        runtime.reset();
        power = 0.075;
        while(runtime.time() < 2.5 && opModeIsActive()){
            robot.updatePoseEstimate();
            //telemetry.addData("Timp ", runtime.time());
            //telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(runtime.time() >= 1.5)
                power = 0;
        }
        stopDriving();

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToSplineHeading(new Pose2d(-13.800778017390152, -51.8,0),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,()->{robot.RidicareBrat(720,0.7); robot.intake.setPower(0.99);})
                .addTemporalMarker(0.7,()->robot.intake.setPower(0))
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1.1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.3,()->{ robot.PivotBrat.setPosition(0.64); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.75); })
                .strafeLeft(0.8)
                .addTemporalMarker(2.0,()-> robot.intake.setPower(0))
                .addTemporalMarker(2.4,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.5)
                .addTemporalMarker(4.8,()->{robot.intake.setPower(0);})
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.6, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.8, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.3,   ()->{robot.RidicareBrat(0,1);})
                .forward(58,SampleMecanumDrive.getVelocityConstraint(50, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .strafeRight(16)
                .build();

        robot.followTrajectorySequence(puneRata);


        if(!opModeIsActive() || isStopRequested())
            stopDriving();

    }

    private void NivelDoi() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();


        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(-60.11550487865194, -58.99897624865089, Math.toRadians(28)),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addTemporalMarker(0,()->{robot.RidicareBrat(200,0.4);})
                .addTemporalMarker(1, ()->{robot.rata.setPower(0.6);})
                .waitSeconds(2)
                .addTemporalMarker(3.2,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);
//323
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(  -25.68310486038491, -45.50910491865442, Math.toRadians(330)),0)
                .addTemporalMarker(0, ()->{robot.RidicareBrat(478,1); })
                .addTemporalMarker(0.2, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(0.3, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(0.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.76); })
                .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.78);})
                .addTemporalMarker(1.7, ()->{robot.intake.setPower(-0.99);})
                .waitSeconds(0.8)
                .addTemporalMarker(2.5, ()->{ robot.intake.setPower(0);robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                .addTemporalMarker(3.0, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.63); })
                .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(3,()->{robot.RidicareBrat(0,1);})
                .splineToSplineHeading(new Pose2d(31.79158659407203, -62.58922924676717, Math.toRadians(0)),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .build(); //daca nu merge cu regionala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

            /*
            shipping hub-> x: -25.68310486038491, y: -49.90910491865442, heading: 330.5245030519176
             */
        robot.followTrajectorySequence(PuneCubPeNivel);

        boolean cub_in_raza = false;
        //merge in fata pana vede ceva

        //      runtime.reset();
//        while (runtime.seconds() < 5) {
//            telemetry.addData("Distanta ", robot.distantaIntake.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
        runtime.reset();
        double power = 0.1;
        while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && runtime.time()<5 && opModeIsActive()){
            robot.updatePoseEstimate();
            telemetry.addData("Timp ", runtime.time());
            telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(robot.distantaIntake.getDistance(DistanceUnit.CM) < 3.4)
                power = 0;
        }
        stopDriving();

//        if(cub_in_raza){
//            runtime.reset();
//            //incercam sa luam cubul, dar daca in 2s nu o luat cubul ne oprim
//            while(robot.distantaIntake.getDistance(DistanceUnit.CM) >=1.3 && runtime.time()<2 && opModeIsActive() )
//                robot.intake.setPower(0.99);
//            stopDriving();
//            if(runtime.time() >= 2 && robot.distantaIntake.getDistance(DistanceUnit.CM) >= 1.3){
//                runtime.reset();
//                //dam robotul in spate
//                while(runtime.time() < 0.9 && opModeIsActive() ) {
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(-0.05);
//                    robot.rightFront.setPower(-0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//                runtime.reset();
//                //dam robotul in stanga
//                while(runtime.time()< 0.7 && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//
//                runtime.reset();
//                //mergem iar in fata si incercam sa luam cubul
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 3.4 && runtime.time()< 3.5  && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(0.05);
//                    robot.intake.setPower(0.99);
//                }
//                stopDriving();
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
//                    robot.intake.setPower(0.99);
//            }
//        }

        //while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
        //robot.intake.setPower(0.99);

        robot.updatePoseEstimate();
        currentPose= robot.getPoseEstimate();

        TrajectorySequence mergeInWh = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(28.004453496583984,-63.46759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-9.800778017390152, -47.99213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(720,0.7);robot.intake.setPower(0);})
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
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.77);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.79);})
                .waitSeconds(0.5)
                .addTemporalMarker(3, ()->{ robot.intake.setPower(-0.99);})
                .waitSeconds(0.2)
                .addTemporalMarker(4,()->{robot.intake.setPower(0);})
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(3.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.4, ()->{robot.RidicareBrat(0,1);})
                .waitSeconds(0.7)
//                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(20))
                .lineToLinearHeading(new Pose2d(1.8218146597307612,-44.804924824164594,Math.toRadians(39)))
                .build();

        robot.followTrajectorySequence(mergeInWh);

        runtime.reset();
        power = 0.075;
        while(runtime.time() < 2.5 && opModeIsActive()){
            robot.updatePoseEstimate();
            //telemetry.addData("Timp ", runtime.time());
            //telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(runtime.time() >= 1.5)
                power = 0;
        }
        stopDriving();

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .lineToSplineHeading(new Pose2d(-13.800778017390152, -51.8,0),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,()->{robot.RidicareBrat(745,0.7);robot.intake.setPower(0.99);})
                .addTemporalMarker(0.7,()->robot.intake.setPower(0))
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1.1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.3,()->{ robot.PivotBrat.setPosition(0.64); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.75); })
                .addTemporalMarker(2.0,()-> robot.intake.setPower(0))
                .addTemporalMarker(2.4,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.2)
                .addTemporalMarker(4.8,()->{robot.intake.setPower(0);})
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.6, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.8, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.3,   ()->{robot.RidicareBrat(0,1);})
                .forward(62,SampleMecanumDrive.getVelocityConstraint(50, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .strafeRight(15.8)
                .build();

        robot.followTrajectorySequence(puneRata);


        if(!opModeIsActive() || isStopRequested())
            stopDriving();
    }

    private void NivelTrei() {

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.updatePoseEstimate();
            Pose2d currentPose = robot.getPoseEstimate();


            TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-60.11550487865194, -58.99897624865089, Math.toRadians(28)),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                            SampleMecanumDrive.getAccelerationConstraint(25))
                    .addTemporalMarker(0,()->{robot.RidicareBrat(720,0.7);})
                    .addTemporalMarker(1, ()->{robot.rata.setPower(0.6);})
                    .waitSeconds(2)
                    .addTemporalMarker(3.2,()->{robot.rata.setPower(0);})
                    .build();

            robot.followTrajectorySequence(RotireRata);
//323
            robot.updatePoseEstimate();
            currentPose = robot.getPoseEstimate();
            TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                    .splineToLinearHeading(new Pose2d(  -25.68310486038491, -45.50910491865442, Math.toRadians(330)),0)
                    .addTemporalMarker(0, ()->{robot.RidicareBrat(720,1); })
                    .addTemporalMarker(0.2, ()->{ robot.PivotBrat.setPosition(0.53); })
                    .addTemporalMarker(0.3, ()->{ robot.PivotBrat.setPosition(0.56); })
                    .addTemporalMarker(0.4, ()->{ robot.PivotBrat.setPosition(0.59); })
                    .addTemporalMarker(0.5, ()->{ robot.PivotBrat.setPosition(0.62); })
                    .addTemporalMarker(0.6, ()->{ robot.PivotBrat.setPosition(0.65); })
                    .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.68); })
                    .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.71); })
                    .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                    .addTemporalMarker(1, ()->{ robot.PivotBrat.setPosition(0.76); })
                    .addTemporalMarker(1.1,()->{robot.PivotBrat.setPosition(0.78);})
                    .addTemporalMarker(1.7, ()->{robot.intake.setPower(-0.99);})
                    .waitSeconds(0.8)
                    .addTemporalMarker(2.5, ()->{ robot.intake.setPower(0);robot.PivotBrat.setPosition(0.74); })
                    .addTemporalMarker(2.6, ()->{ robot.PivotBrat.setPosition(0.73); })
                    .addTemporalMarker(2.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                    .addTemporalMarker(2.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                    .addTemporalMarker(2.9, ()->{ robot.PivotBrat.setPosition(0.67); })
                    .addTemporalMarker(3.0, ()->{ robot.PivotBrat.setPosition(0.65); })
                    .addTemporalMarker(3.1, ()->{ robot.PivotBrat.setPosition(0.63); })
                    .addTemporalMarker(3.2, ()->{ robot.PivotBrat.setPosition(0.61); })
                    .addTemporalMarker(3.3, ()->{ robot.PivotBrat.setPosition(0.59); })
                    .addTemporalMarker(3.4, ()->{ robot.PivotBrat.setPosition(0.57); })
                    .addTemporalMarker(3.5, ()->{ robot.PivotBrat.setPosition(0.55); })
                    .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.53); })
                    .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.5); })
                    .addTemporalMarker(3,()->{robot.RidicareBrat(0,1);})
                    .splineToSplineHeading(new Pose2d(31.79158659407203, -62.58922924676717, Math.toRadians(0)),0 , SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                            SampleMecanumDrive.getAccelerationConstraint(50))
                    .build(); //daca nu merge cu regionala facem battle bots in parcu teilor #Iftime Mihail Kogalniceanu

            /*
            shipping hub-> x: -25.68310486038491, y: -49.90910491865442, heading: 330.5245030519176
             */
            robot.followTrajectorySequence(PuneCubPeNivel);

            boolean cub_in_raza = false;
            //merge in fata pana vede ceva

  //      runtime.reset();
//        while (runtime.seconds() < 5) {
//            telemetry.addData("Distanta ", robot.distantaIntake.getDistance(DistanceUnit.CM));
//            telemetry.update();
//        }
        runtime.reset();
        double power = 0.1;
        while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && runtime.time()<5 && opModeIsActive()){
            robot.updatePoseEstimate();
            telemetry.addData("Timp ", runtime.time());
            telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(robot.distantaIntake.getDistance(DistanceUnit.CM) < 3.4)
                power = 0;
        }
        stopDriving();

//        if(cub_in_raza){
//            runtime.reset();
//            //incercam sa luam cubul, dar daca in 2s nu o luat cubul ne oprim
//            while(robot.distantaIntake.getDistance(DistanceUnit.CM) >=1.3 && runtime.time()<2 && opModeIsActive() )
//                robot.intake.setPower(0.99);
//            stopDriving();
//            if(runtime.time() >= 2 && robot.distantaIntake.getDistance(DistanceUnit.CM) >= 1.3){
//                runtime.reset();
//                //dam robotul in spate
//                while(runtime.time() < 0.9 && opModeIsActive() ) {
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(-0.05);
//                    robot.rightFront.setPower(-0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//                runtime.reset();
//                //dam robotul in stanga
//                while(runtime.time()< 0.7 && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(-0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(-0.05);
//                }
//                stopDriving();
//
//                runtime.reset();
//                //mergem iar in fata si incercam sa luam cubul
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 3.4 && runtime.time()< 3.5  && opModeIsActive() ){
//                    robot.updatePoseEstimate();
//                    currentPose = robot.getPoseEstimate();
//                    robot.leftFront.setPower(0.05);
//                    robot.leftRear.setPower(0.05);
//                    robot.rightFront.setPower(0.05);
//                    robot.rightRear.setPower(0.05);
//                    robot.intake.setPower(0.99);
//                }
//                stopDriving();
//                while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
//                    robot.intake.setPower(0.99);
//            }
//        }

        //while(robot.distantaIntake.getDistance(DistanceUnit.CM) > 1.6 && opModeIsActive() )
            //robot.intake.setPower(0.99);

        robot.updatePoseEstimate();
        currentPose= robot.getPoseEstimate();

        TrajectorySequence mergeInWh = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(28.004453496583984,-63.46759694447507),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .splineToConstantHeading(new Vector2d(-10.800778017390152, -47.99213093601243),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,   ()->{ robot.RidicareBrat(720,0.7);robot.intake.setPower(0);})
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
                .addTemporalMarker(2,()->{robot.PivotBrat.setPosition(0.77);})
                .addTemporalMarker(2.1,()->{robot.PivotBrat.setPosition(0.79);})
                .waitSeconds(0.5)
                .addTemporalMarker(3, ()->{ robot.intake.setPower(-0.99);})
                .waitSeconds(0.2)
                .addTemporalMarker(4,()->{robot.intake.setPower(0);})
                .addTemporalMarker(3.6, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(3.7, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(3.8, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.6, ()->{robot.RidicareBrat(0,1);})
                .waitSeconds(0.7)
//                .splineToConstantHeading(new Vector2d(38.79158659407203, -62.58922924676717),0,SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
//                        SampleMecanumDrive.getAccelerationConstraint(20))
                .lineTo(new Vector2d(6.455264826189605,-36.5))
                .build();

        robot.followTrajectorySequence(mergeInWh);

        runtime.reset();
        power = 0.075;
        while(runtime.time() < 2.5 && opModeIsActive()){
            robot.updatePoseEstimate();
            //telemetry.addData("Timp ", runtime.time());
            //telemetry.update();
            robot.leftFront.setPower(power);
            robot.leftRear.setPower(power);
            robot.rightFront.setPower(power);
            robot.rightRear.setPower(power);
            robot.intake.setPower(0.99);

            if(runtime.time() >= 1.5)
                power = 0;
        }
        stopDriving();

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence puneRata = robot.trajectorySequenceBuilder(currentPose)
                .back(0.001)
                .lineToConstantHeading(new Vector2d(-13.800778017390152, -51.8),SampleMecanumDrive.getVelocityConstraint(40, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .addTemporalMarker(0,()->{robot.RidicareBrat(720,0.7);robot.intake.setPower(0.99);})
                .addTemporalMarker(0.7,()->robot.intake.setPower(0))
                .addTemporalMarker(0.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(0.8, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(0.9, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(1.0, ()->{ robot.PivotBrat.setPosition(0.58); })
                .addTemporalMarker(1.1,   ()->{ robot.PivotBrat.setPosition(0.60); })
                .addTemporalMarker(1.2, ()->{ robot.PivotBrat.setPosition(0.62); })
                .addTemporalMarker(1.3,()->{ robot.PivotBrat.setPosition(0.64); })
                .addTemporalMarker(1.4, ()->{ robot.PivotBrat.setPosition(0.66); })
                .addTemporalMarker(1.5, ()->{ robot.PivotBrat.setPosition(0.68); })
                .addTemporalMarker(1.6, ()->{ robot.PivotBrat.setPosition(0.70); })
                .addTemporalMarker(1.7, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(1.8, ()->{ robot.PivotBrat.setPosition(0.73); })
                .addTemporalMarker(1.9, ()->{ robot.PivotBrat.setPosition(0.74); })
                .addTemporalMarker(2.0, ()->{ robot.PivotBrat.setPosition(0.75); })
                .strafeLeft(1.2)
                .addTemporalMarker(2.0,()-> robot.intake.setPower(0))
                .addTemporalMarker(2.4,()->{robot.intake.setPower(-0.99);})
                .waitSeconds(1.2)
                .addTemporalMarker(3.9, ()->{ robot.PivotBrat.setPosition(0.72); })
                .addTemporalMarker(4, ()->{ robot.PivotBrat.setPosition(0.71); })
                .addTemporalMarker(4.1, ()->{ robot.PivotBrat.setPosition(0.69); })
                .addTemporalMarker(4.2, ()->{ robot.PivotBrat.setPosition(0.77); })
                .addTemporalMarker(4.3, ()->{ robot.PivotBrat.setPosition(0.65); })
                .addTemporalMarker(4.4, ()->{ robot.PivotBrat.setPosition(0.60);robot.intake.setPower(0); })
                .addTemporalMarker(4.5, ()->{ robot.PivotBrat.setPosition(0.56); })
                .addTemporalMarker(4.6, ()->{ robot.PivotBrat.setPosition(0.54); })
                .addTemporalMarker(4.7, ()->{ robot.PivotBrat.setPosition(0.52); })
                .addTemporalMarker(4.8, ()->{ robot.PivotBrat.setPosition(0.5); })
                .addTemporalMarker(4.3,   ()->{robot.RidicareBrat(0,1);})
                .forward(58,SampleMecanumDrive.getVelocityConstraint(50, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(50))
                .strafeRight(16)
                .build();

        robot.followTrajectorySequence(puneRata);


        if(!opModeIsActive() || isStopRequested())
            stopDriving();
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
