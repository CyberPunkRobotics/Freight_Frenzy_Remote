package org.firstinspires.ftc.teamcode.drive.teleop;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.autonomii_camera.AutonomieFinala;
import org.firstinspires.ftc.teamcode.drive.properties.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.properties.StandardTrackingWheelLocalizer;

import static org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.StdCatchBuilder.build;


@TeleOp(name = "RedTeleOp", group = "MecanumBot")
public class RedTeleOp extends LinearOpMode {

    private SampleMecanumDrive robot = null;
    private final double DELAY = 0.2;
    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime brat = new ElapsedTime();

    boolean ridicare_brat = false;
    boolean capping_brat_take = false;
    boolean capping_brat_ridicare = false;
    boolean o_ajuns = false;

    double power=0.6;
    int k = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot = new SampleMecanumDrive(hardwareMap);

        //encodere pt ridicare brat
        robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.cap.setPosition(0);

        //robot in pozitie de start
        robot.setPoseEstimate(new Pose2d(34.434152253040736,-63.030941829754916));

        //contor tickuri ridicare/coborare brat
        int ticks;

        //pt determinare pozitie robot (la inceput e inafara warehouse-ului)
        //warehouse = false , sh = true
        String pozitie = "warehouse";

        String cfbrat = "default";

        //telemetry pe dashboard
        Telemetry dashboardTelemetry;
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //pozitiile inititale ale bratului + clestelui +cap
        //robot.intake.setPosition(0.23);
        robot.PivotBrat.setPosition(0.5);

        waitForStart();

        brat.reset();

        robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int k =0;

        while (opModeIsActive()) {


            robot.updatePoseEstimate();

            //contorizare tick-uri
            ticks = robot.ridicareBrat.getCurrentPosition();

//            runtime.reset();
            /** GAMEPAD 1 */

            //Senzori distanta de la senzori culoare
            double dD = robot.distantaDreapta.getDistance(DistanceUnit.CM);
            double dS = robot.distantaStanga.getDistance(DistanceUnit.CM);
            double dI = robot.distantaIntake.getDistance(DistanceUnit.CM);


            //Miscarea sasiului

             double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double direction = Math.atan2(x, y) - Math.toRadians(robot.getAngle()) + Math.PI / 2;
            double ipotenuse = Math.sqrt(x * x + y * y);
            double rotate = gamepad1.right_stick_x * 0.50;
            double strafe = Math.sin(direction) * ipotenuse;
            double forward = Math.cos(direction) * ipotenuse;

            double lR = -power * robot.SQRT(-strafe - forward - rotate);
            double rF = power * robot.SQRT(strafe + forward - rotate);
            double lF = -power * robot.SQRT(strafe - forward - rotate);
            double rR = power * robot.SQRT(-strafe + forward - rotate);



//            //cand esti in warehouse te misti mai incet
//            if (!pozitie) {
//                power = lowPower;
//                lR = -power * robot.SQRT(-strafe - forward - rotate);
//                rF = power * robot.SQRT(strafe + forward - rotate);
//                lF = -power * robot.SQRT(strafe - forward - rotate);
//                rR = power * robot.SQRT(-strafe + forward - rotate);
//            }
//            else {
//                power = highPower;
//            }

            robot.setMotorPowers(lF, lR, rR, rF);

//            pozitie = !(robot.getPoseEstimate().getX() > 28) || !(robot.getPoseEstimate().getY() > -70);

            if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x == 0){
                robot.stopMotors();
        }



            //senzori culoare
            double rStanga = robot.culoareSpate.red();
            double gStanga = robot.culoareSpate.green();
            double bStanga = robot.culoareSpate.blue();



            //actionare intake
            if(gamepad1.right_bumper && dI > 1.2)
                robot.intake.setPower(0.99);
            else if(gamepad1.left_bumper)
                robot.intake.setPower(-0.99);
            else robot.intake.setPower(0);


            //Resetare unghi
            if (gamepad1.square) {
                robot.resetAngle();
            }

//            //Resetare pozitii warehouse
//            if (robot.culoareSpate.getNormalizedColors().toColor() > 1730000000)
//                robot.setPoseEstimate(new Pose2d(28.594453496583984, -62.96759694447507));
//
//            //capping
//            if(gamepad1.dpad_up)
//                robot.cap.setPosition(1);
//            else if(gamepad1.dpad_down)
//                robot.cap.setPosition(0);

            /* CHESTII DE COD PE CARE POATE LE FOLOSIM*/
            //pozitionare robot paralel cu peretele (aliniare teren)
//            if(gamepad1.dpad_right){
//              // robot.updatePoseEstimate();
//                    robot.turn(Math.toRadians(-(robot.getAngle())));
//            }
//
//
//            //Deschidere/Inchidere Cleste
//            if (gamepad1.right_bumper) {
//                robot.intake.setPosition(0.32);
//            }
//            if (gamepad1.left_bumper) {
//                robot.intake.setPosition(0.23);
//            }
//
//
//            //resetare unghi la peste 360
//            if (robot.getAngle() > 360 || robot.getAngle() < -360)
//                robot.resetAngle();
//
//            //pozitionare robot paralel cu peretele (aliniare teren)
//            if(gamepad1.dpad_right){
//              // robot.updatePoseEstimate();
//                    robot.turn(Math.toRadians(-(robot.getAngle())));
//            }
            /// alb v mica 119 184 210
            ///rosu v mica 195 155 196
            /// alb v mare 100-150 , 150, 160-200
            /// rosu v mare 100,160, 180
            //detectarea culorii albe
//            daca detecteaza culoarea alba => ai trecut peste linia dintre warehouse si shipping hub
//            if (robot.culoareSpate.red() >= 100 && robot.culoareSpate.green() >= 150 && robot.culoareSpate.blue() >= 150 && robot.culoareSpate.alpha() >= 90) {
//            //if (robot.culoareSpate.getNormalizedColors().toColor() > 1850000000) {
//                if (pozitie == "warehouse" && time.time() > 1.512) {
//                    pozitie = "shipping_hub";
//                    time.reset();
//                } else if (pozitie =="shipping_hub" && time.time() > 1.512) {
//                  pozitie = "warehouse";
//                  time.reset();
//                }
//                //pentru a nu pierde din pozitia robotului, ne folosim de coordonatele
//                //liniei albe si resetam pozitia robotului pentru a stii unde se afla
//                robot.setPoseEstimate(new Pose2d(28.594453496583984, -62.96759694447507));
//            }
//            if (robot.culoareFata.getNormalizedColors().toColor() > 1250000000) {
//                if (!pozitie && time.time() > 1.512) {
//                    pozitie = true;
//                    time.reset();
//                } else if (pozitie && time.time() > 1.512) {
//                    pozitie = false;
//                    time.reset();
//                }
//                //pentru a nu pierde din pozitia robotului, ne folosim de coordonatele
//                //liniei albe si resetam pozitia robotului pentru a stii unde se afla
//                robot.setPoseEstimate(new Pose2d(28.594453496583984, -62.96759694447507));
//            }



            /** GAMEPAD 2 */

            //actionare brat
            double bS,bJ,bS2, bJ2;

            double putere_brat = 1;
            bS = putere_brat* gamepad2.right_trigger;
            bJ = putere_brat* gamepad2.left_trigger;
            bS2 = 0.3 * gamepad1.left_trigger;
            bJ2 = 0.3 * gamepad1.right_trigger;

            if(bS > 0)
                robot.ridicareBrat.setPower(bS);
            else if(bJ > 0)
                robot.ridicareBrat.setPower(-bJ);
            else if(bS2 > 0)
                robot.ridicareBrat.setPower(bS2);
            else if(bJ2 > 0)
                robot.ridicareBrat.setPower(-bJ2);
            else if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0 && gamepad1.right_trigger==0 && gamepad1.left_trigger == 0 &&
                    !ridicare_brat && !capping_brat_take) robot.ridicareBrat.setPower(0);

            //ridicare brat autonoma

                  //nivel 3 cu dpad
            if (gamepad2.dpad_up){
                ridicare_brat = true;
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setPower(1);
                robot.ridicareBrat.setTargetPosition(700);
            }
            if(ridicare_brat && (ticks >= 700 || gamepad2.right_trigger> 0 || gamepad2.left_trigger > 0
                    || gamepad1.right_trigger>0 || gamepad1.left_trigger>0)){
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.ridicareBrat.setPower(0);
                ridicare_brat=false;
                o_ajuns = true;
            }
                  //nivel 3 cu sensor
            if(dI<=1.2 && !o_ajuns && ticks < 700 && gamepad1.left_trigger==0 && gamepad1.right_trigger == 0
                && gamepad2.left_trigger==0 && gamepad2.right_trigger==0)
            {ridicare_brat = true;
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setPower(1);
                robot.ridicareBrat.setTargetPosition(700);
            }
            if(dI>2.5)
                o_ajuns = false;

//
//            //resetare pozitii brat cu touch sensor.
//            if(robot.touchSensor.isPressed() )
//                    //&& !ridicare_brat && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0
////                    && gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 )
//            {
//                   k++;
//                    robot.ridicareBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//         else k =0;

         //restare tick-uri
            if(gamepad2.square)
            {robot.ridicareBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}

            if(robot.touchSensor.isPressed() && k<1)
            {robot.ridicareBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);k++;
            }else k=0;
//
//            //nivel capping-take
//            if(gamepad2.triangle){
//                capping_brat_take = true;
//                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.ridicareBrat.setPower(0.5);
//                robot.ridicareBrat.setTargetPosition(205);
//            }
//            if(capping_brat_take && (robot.ridicareBrat.getCurrentPosition() >=205  || robot.ridicareBrat.getCurrentPosition() <=-10)){
//                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.ridicareBrat.setPower(0);
//                capping_brat_take=false;
//            }

//            //nivel capping-ridicare
//            if(gamepad2.right_stick_button){
//                capping_brat_ridicare = true;
//                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.ridicareBrat.setPower(0.5);
//                robot.ridicareBrat.setTargetPosition(900);
//            }
//            if(capping_brat_ridicare && (robot.ridicareBrat.getCurrentPosition() >=900  || robot.ridicareBrat.getCurrentPosition() <=-10)){
//                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.ridicareBrat.setPower(0);
//                capping_brat_take=false;
//            }

            //carusel //da
            if(gamepad2.right_bumper)
                robot.rata.setPower(0.6);
//            else if(gamepad2.left_bumper)
//                robot.rata.setPower(-0.1);
            else {robot.rata.setPower(0);robot.rata.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);}

            //rotatie brat
            if(gamepad2.dpad_left) {
                if(robot.PivotBrat.getPosition()<=0.77)
                    robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()+0.03);
            }
            if(gamepad2.dpad_right){
                if(robot.PivotBrat.getPosition()>=0.50)
                    robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()-0.03);
            }

            if(gamepad2.right_stick_x < 0)
                robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()+0.01);
            if(gamepad2.right_stick_x > 0)
                robot.PivotBrat.setPosition(robot.PivotBrat.getPosition()-0.01);

            if(brat.time() == 2)
            {gamepad1.rumble(100);gamepad2.rumble(100);}

            if(gamepad2.left_stick_button)
                power=1;
            if(gamepad2.right_stick_button)
                power=0.6;


            /* CHESTII DE COD PE CARE POATE LE FOLOSIM*/
/*
            //reducere viteza
            //     if(gamepad1.right_stick_button ) {
//                if (!pozitie)
//                    pozitie = true;
//                else if (pozitie)
//                    pozitie = false;
//                time.reset();
//            }

           if(gamepad2.dpad_right)
            if(gamepad2.x) {
                double p = 0.5;
                while(p<=0.75) {
                    robot.PivotBrat.setPosition(p);
                    sleep(100);
                    p+=0.02;
                }
            }

            ///nivele brat

            nivel 1 = 380 ticks
            nivel 2 = 860 ticks
            nivel 3 = 1280 ticks

            /*if(gamepad2.dpad_up) {
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setTargetPosition(1280);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                nivel_brat = "sus";
            }
                if(nivel_brat == "sus" && robot.ridicareBrat.getCurrentPosition()<=1280) {
                    robot.ridicareBrat.setPower(1);
                }

            if(gamepad2.dpad_down){
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.ridicareBrat.setTargetPosition(0);
                robot.ridicareBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    nivel_brat = "jos";
            }
                if(nivel_brat == "jos" && robot.ridicareBrat.getCurrentPosition()>=0) {
                    robot.ridicareBrat.setPower(1);
                }

                if(robot.ridicareBrat.getCurrentPosition() == 1280 || robot.ridicareBrat.getCurrentPosition() == 0)
                    nivel_brat = "-";

*/


            dashboardTelemetry.addData("ticks:", ticks);
            dashboardTelemetry.addData("Color intake ", robot.culoareIntake.getNormalizedColors().toColor());
            dashboardTelemetry.addData("Color intake: ","R %d G %d B %d A %d", robot.culoareIntake.red(), robot.culoareIntake.blue(), robot.culoareIntake.green(), robot.culoareIntake.alpha());
            dashboardTelemetry.addData("Color spate ", robot.culoareSpate.getNormalizedColors().toColor());
            dashboardTelemetry.addData("Color spate","R %d G %d B %d", robot.culoareSpate.red(), robot.culoareSpate.blue(), robot.culoareSpate.green(),robot.culoareSpate.alpha());
            dashboardTelemetry.addData("x", robot.getPoseEstimate().getX());
            dashboardTelemetry.addData("y", robot.getPoseEstimate().getY());
            dashboardTelemetry.addData("Pozitie robot",pozitie);
            dashboardTelemetry.addData("distanta intake", robot.distantaIntake.getDistance(DistanceUnit.CM));
            dashboardTelemetry.addData("Brat mode", robot.ridicareBrat.getMode());
            dashboardTelemetry.addData("Touch Sensor", robot.touchSensor.isPressed());
//            if(robot.culoareIntake.alpha() > 2000)
//                dashboardTelemetry.addData("In intake: ","CUB");
//            else if (robot.culoareIntake.alpha() < 2000 && dI < 2)
//                dashboardTelemetry.addData("In intake: ","MINGE");
//            else dashboardTelemetry.addData("In intake: ","NIMIC");

            dashboardTelemetry.update();
        }

    }
}