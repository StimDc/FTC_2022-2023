/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="tentativa lui rares", group="Linear Opmode")
public class TentativaLuiRares extends LinearOpMode
{
    private DcMotor front_left=null;
    private DcMotor front_right=null;
    private DcMotor back_left=null;
    private DcMotor back_right=null;
    private DcMotor slider=null;
    private DcMotor rotatory_base = null;
    private Servo c1 ,c2 = null;
    private ElapsedTime runtime = new ElapsedTime();

    private String direction = "straight";
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int[] tags = {17,18,19};
    String[] directions = {"left","straight","right"};
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Test");
        telemetry.update();

        //Links the virtual objets to the real motors
        front_left=hardwareMap.get(DcMotor.class, "FL");
        front_right=hardwareMap.get(DcMotor.class, "FR");
        back_left=hardwareMap.get(DcMotor.class, "BL");
        back_right=hardwareMap.get(DcMotor.class, "BR");
        slider=hardwareMap.get(DcMotor.class, "SL");
        rotatory_base = hardwareMap.get(DcMotor.class,"RB");
        c1 = hardwareMap.get(Servo.class, "C1");
        c2 = hardwareMap.get(Servo.class,"C2");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.REVERSE);

        rotatory_base.setDirection(DcMotor.Direction.REVERSE);

        autonomie robot = new autonomie(front_left,front_right,back_left,back_right,rotatory_base,slider,c1,c2);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, robot.camera_fx, robot.camera_fy, robot.camera_cx, robot.camera_cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    for(int i =0;i<3;i++){
                        if(tag.id == tags[i]){
                            direction = directions[i];
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            autonomie(robot);
        }
        else
        {
            autonomie(robot);



        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }
    public void autonomie(autonomie robot){
        robot.runUsingEncoders();
        robot.setAllPower(0.6,0.6,0.5);
        robot.resetEncoders();

        robot.initSlider();

        robot.firstConeBigPole("right");



        robot.resetEncodersSlider();
        robot.runUsingEncodersSlider();
        robot.stopSlider();

        robot.resetEncodersWheelMotors();
        robot.lateral(70,robot.getWheelPower(),"right");
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.resetEncodersWheelMotors();
        robot.forward(-10,robot.getWheelPower());
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.resetEncodersWheelMotors();
        robot.lateral(16,robot.getWheelPower(),"right");
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.releaseCone();

        robot.resetEncodersSlider();
        robot.sliderUp(15,robot.getSliderPower());
        robot.waitSlider();

        robot.resetEncodersRotatoryBase();
        robot.rotateRotatoryBase(110,robot.getWheelPower(),"right");
        robot.waitRotatoryBase();
        robot.stopRotatoryBase();

        robot.grabCone();
        sleep(400);

        robot.resetEncodersSlider();
        robot.sliderUp(15,robot.getSliderPower());
        robot.waitSlider();

        robot.resetEncodersRotatoryBase();
        robot.rotateRotatoryBase(110,robot.getWheelPower(),"left");
        robot.waitRotatoryBase();
        robot.stopRotatoryBase();




        robot.resetEncodersWheelMotors();
        robot.lateral(16,robot.getWheelPower(),"left");
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.resetEncodersWheelMotors();
        robot.forward(11,robot.getWheelPower());
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.resetEncodersWheelMotors();
        robot.lateral(50,robot.getWheelPower(),"left");
        robot.waitWheelMotors();
        robot.stopWheelMotors();

        robot.resetEncodersSlider();
        robot.sliderUp(60,robot.getSliderPower());
        robot.waitSlider();

        robot.resetEncodersWheelMotors();
        robot.lateral(20,robot.getWheelPower(),"left");
        robot.waitWheelMotors();
        robot.stopWheelMotors();


        robot.resetEncodersRotatoryBase();
        robot.rotateRotatoryBase(51,robot.getRotBasePower(),"left");
        robot.waitRotatoryBase();
        robot.stopRotatoryBase();

        sleep(500);
        robot.releaseCone();

        robot.resetEncodersRotatoryBase();
        robot.rotateRotatoryBase(51,robot.getRotBasePower(),"right");
        robot.waitRotatoryBase();
        robot.stopRotatoryBase();







    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}