package org.firstinspires.ftc.teamcode.drive.autonomii_camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
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
        ElapsedTime timer = new ElapsedTime();

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

            case TREI:



        }
        webcam.stopStreaming();
    }

    private void NivelUnu()
    {
        robot.updatePoseEstimate();

        Pose2d currentPose = robot.getPoseEstimate();
        Trajectory traj1 = robot.trajectoryBuilder(currentPose,true)
                .lineTo(new Vector2d(-59.84777435756638,-59.84777435756638))
//                .addSpatialMarker(new Vector2d(-59.84777435756638,-59.84777435756638), ()->{
//                    robot.rata.setPower(1);
//                })
                .addTemporalMarker(1.3,()->{
                    robot.rata.setPower(1);
                })
                .addTemporalMarker(3, ()->{
                    robot.rata.setPower(0);
                })
                .build();

        robot.followTrajectory(traj1);
        robot.rata.setPower(0);

        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();


        robot.updatePoseEstimate();

        Trajectory traj2 = robot.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d( -8.345234014858367, -45.88761111604068),0)
                .addTemporalMarker(0, () -> {


                    robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.ridicareBrat.setTargetPosition(380);
                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(robot.ridicareBrat.isBusy() )
                        robot.ridicareBrat.setPower(1);


                })
                .addTemporalMarker(4, () ->{
                    double p = 0.5;
                    while (p<=0.75) {
                        robot.PivotBrat.setPosition(p);
                        p+=0.02;
                    }

                })
               .addSpatialMarker(new Vector2d(-8.345234014858367, -45.88761111604068), ()->{
                   robot.cleste.setPosition(0.5);
                })
                .build();

        robot.followTrajectory(traj2);
        robot.updatePoseEstimate();
        currentPose = robot.getPoseEstimate();
    }

    }
