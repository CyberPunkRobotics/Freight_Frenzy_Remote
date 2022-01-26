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

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new SampleMecanumDrive(hardwareMap);

        robot.setPoseEstimate(new Pose2d(-41.862855949849056,-63.11163142092735));
        robot.setExternalHeading(Math.toRadians(0));

        dashboard = FtcDashboard.getInstance();

        robot.cleste.setPosition(0.6);
        robot.PivotBrat.setPosition(0.5);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectarePozitie(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                           @Override
                                           public void onOpened() {
                                               webcam.startStreaming(320,240,OpenCvCameraRotation.SIDEWAYS_LEFT);
                                               dashboard.startCameraStream(webcam,120);
                                           }

                                           @Override
                                           public void onError(int errorCode) {

                                           }
                                       }

        );


        waitForStart();

        telemetry.addData("unghi",robot.getRawExternalHeading());

        switch (pipeline.getLocation()) {
            case UNU:
                NivelUnu();

            case DOI:
                break;

            case TREI:
                break;



        }
        webcam.stopStreaming();
    }

    private void NivelUnu()
    {
        robot.updatePoseEstimate();
        Pose2d currentPose = robot.getPoseEstimate();


        TrajectorySequence RotireRata = robot.trajectorySequenceBuilder(currentPose)
                .lineTo(new Vector2d(-60.11550487865194, -58.99897624865089),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .addTemporalMarker(1, ()->{robot.rata.setPower(1);})
                .waitSeconds(2)
                .addTemporalMarker(3.2,()->{robot.rata.setPower(0);})
                .build();

        robot.followTrajectorySequence(RotireRata);

        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
        TrajectorySequence PuneCubPeNivel = robot.trajectorySequenceBuilder(currentPose)
                .splineTo(new Vector2d(-11.922752632451449,-45.67213093601243),0)
                .addTemporalMarker(0, ()->{
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(314);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(robot.ridicareBrat.isBusy() ) {
                        robot.ridicareBrat.setPower(1);
                    }
                })
                .addDisplacementMarker(43,()->{robot.PivotBrat.setPosition(0.75);})
                .addTemporalMarker(9.2, ()->{robot.cleste.setPosition(0.5);})
                .waitSeconds(0.5)
                .build();

        robot.followTrajectorySequence(PuneCubPeNivel);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence MergeInWarehouse = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(5.3, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(36))
                .addTemporalMarker(0.5, ()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(0.4, ()->{
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(0);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(robot.ridicareBrat.isBusy() ) {
                        robot.ridicareBrat.setPower(1);
                    }})
                .splineToConstantHeading(new Vector2d(35.741806061837597,-63.35317556426468),0,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

         robot.followTrajectorySequence(MergeInWarehouse);
         robot.updatePoseEstimate();
         currentPose= robot.getPoseEstimate();

         TrajectorySequence IaCub = robot.trajectorySequenceBuilder(currentPose)
                 .forward(5, SampleMecanumDrive.getVelocityConstraint(30, 6,10),
                 SampleMecanumDrive.getAccelerationConstraint(10))
                 .addDisplacementMarker(5, ()->{robot.cleste.setPosition(0.6);})
                 .lineTo(new Vector2d(28.594453496583984,-62.96759694447507),SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                         SampleMecanumDrive.getAccelerationConstraint(7))
                 .build();




         robot.followTrajectorySequence(IaCub);
         robot.updatePoseEstimate();
         currentPose = robot.getPoseEstimate();

        TrajectorySequence MergeLaSH = robot.trajectorySequenceBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(-11.922752632451449,-45.67213093601243),0)
                .addDisplacementMarker(6, ()->{robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(1280);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(robot.ridicareBrat.isBusy() ) {
                        robot.ridicareBrat.setPower(1);
                    }})
                .addDisplacementMarker(27, ()->{robot.PivotBrat.setPosition(0.75);})
                .addTemporalMarker(7, ()->{robot.cleste.setPosition(0.5);})
                .waitSeconds(0.5)
                .build();

        robot.followTrajectorySequence(MergeLaSH);
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();

        TrajectorySequence MergeInWarehouse2 = robot.trajectorySequenceBuilder(currentPose)
                .strafeRight(5.3, SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(36))
                .addTemporalMarker(0.5, ()->{robot.PivotBrat.setPosition(0.5);})
                .addTemporalMarker(0.4, ()->{
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(0);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(robot.ridicareBrat.isBusy() ) {
                        robot.ridicareBrat.setPower(1);
                    }})
                .splineToConstantHeading(new Vector2d(35.741806061837597,-63.35317556426468),0,SampleMecanumDrive.getVelocityConstraint(62.01654253906262, 5.788888931274414,10),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        robot.followTrajectorySequence(MergeInWarehouse2);
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();


    }

}
