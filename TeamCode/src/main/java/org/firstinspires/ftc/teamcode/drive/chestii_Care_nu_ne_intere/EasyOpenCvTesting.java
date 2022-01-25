/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.chestii_Care_nu_ne_intere;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Config
@Autonomous(name="Autonomie finala")
public class EasyOpenCvTesting extends LinearOpMode
{
    OpenCvCamera webcam;
    DectectareTSE pipeline;
    SampleMecanumDrive robot;

    /*
     * The core values which define the location and size of the sample regions
     */
    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70,105);
    public static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(209,110);
    public static int checkIndex = 0;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    public static int REGION_WIDTH = 10;
    public static int REGION_HEIGHT = 10;

//    public static int FOUR_RING_THRESHOLD = 135;
//    public static int ONE_RING_THRESHOLD = 135;
    public static int THRESHOLD = 135;

//    private final double TOWER_GOAL_HEIGHT = 0.55;

    @Override
    public void runOpMode()
    {
        robot = new SampleMecanumDrive(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DectectareTSE();
        webcam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            pipeline.updateRegions();
        }

        waitForStart();

        if (opModeIsActive())
        {

            pipeline.updateRegions();

            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();


            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.position == DectectareTSE.pozitie.UNU) {

                sleep(100);

            }
                else if (pipeline.position == DectectareTSE.pozitie.DOI){

                sleep(100);

            }
            else if(pipeline.position == DectectareTSE.pozitie.TREI){

                sleep(100);

            }

        }
    }





    public static class DectectareTSE extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum pozitie
        {
            UNU,
            DOI,
            TREI
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 0, 0, 0);




        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        public void updateRegions() {
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile pozitie position = pozitie.UNU;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[checkIndex];
            avg2 = (int) Core.mean(region2_Cb).val[checkIndex];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = pozitie.UNU; // Record our analysis
            if(avg1 > THRESHOLD && avg2 >THRESHOLD){
                position = pozitie.UNU;
            }else if (avg1 > THRESHOLD && avg2 < THRESHOLD){
                position = pozitie.DOI;
            }else{
                position = pozitie.TREI;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}