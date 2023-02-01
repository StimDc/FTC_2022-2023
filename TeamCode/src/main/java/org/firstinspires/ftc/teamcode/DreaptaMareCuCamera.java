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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="Dreapta mare cu camera", group="Linear Opmode")
public class DreaptaMareCuCamera extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    

    // UNITS ARE METERS
    double tagsize = 0.166;
    
     // Tag ID 18 from the 36h11 family
    int[] TAGS = {17,18,19};
    String id;
    AprilTagDetection tagOfInterest = null;
    String direction;
    private DcMotor front_left=null;
    private DcMotor front_right=null;
    private DcMotor back_left=null;
    private DcMotor back_right=null;
    private DcMotor slider=null;
    private DcMotor rotatory_base = null;
    private DcMotor arm = null;
    private Servo c1 = null;
    private Servo c2 = null;
    @Override
    public void runOpMode()
    {
        front_left=hardwareMap.get(DcMotor.class, "FL");
        front_right=hardwareMap.get(DcMotor.class, "FR");
        back_left=hardwareMap.get(DcMotor.class, "BL");
        back_right=hardwareMap.get(DcMotor.class, "BR");
        slider=hardwareMap.get(DcMotor.class, "SL");
        rotatory_base = hardwareMap.get(DcMotor.class,"RB");
        arm = hardwareMap.get(DcMotor.class, "AR");
        c1 = hardwareMap.get(Servo.class, "C1");
        c2 = hardwareMap.get(Servo.class, "C2");
        StimDC robot = new StimDC(front_left,front_right,back_left,back_right,rotatory_base,slider,arm,c1,c2);
        //robot.hardware(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, robot.camera_fx, robot.camera_fy, robot.camera_cx, robot.camera_cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){
                
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
                    for(int i = 0;i<3;i++){
                        if( tag.id == TAGS[i]){
                            if(tag.id == 17){
                                direction = "left";
                            }
                            else if(tag.id == 18){
                                direction = "straight";
                            }
                            else{
                                direction = "right";
                            }
                            
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                    
                }

                if(tagFound == true)
                {
                    telemetry.addLine("Image was found ");
                   
                }
                else
                {
                    telemetry.addLine("NO IMAGE WAS FOUND YOU MAY CHANGE AUTONOMOUS");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("REALLY CHANGE THE AUTONOMOUS");
                    }
                    else
                    {
                        //robot.telemetry.addLine("its good lmao");
                        //robot.tagToTelemetry(tagOfInterest);
                    }
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
            autonomie(robot);
            sleep(30000);


       
    }
    public void autonomie(StimDC robot){
        robot.run_using_encoders();
            
        robot.reset_encoders();
        robot.forward(130,0.5);
        robot.wait_motors();
        robot.stop();
            
        robot.reset_encoders();
        robot.lateral(30,0.5,"right");
        robot.wait_motors();
        robot.stop();
            
        robot.run_using_encoders();
        robot.reset_encoders();
        robot.rotate_rotatory_base(65,0.6,"left");
        robot.wait_motors();
        robot.stop();
        sleep(1000);
        for(int i = 1;i<=5;i++){
           robot.reset_encoders();
            robot.rotate_rotatory_base(170,0.6,"right");
            robot.wait_motors();
            robot.stop();
            sleep(1000); 
            
            robot.reset_encoders();
            robot.rotate_rotatory_base(170,0.6,"left");
            robot.wait_motors();
            robot.stop();
            sleep(1000);
        }
        
        robot.run_using_encoders();
        robot.reset_encoders();
        robot.lateral(30,0.5,"left");
        robot.wait_motors();
        robot.stop();
        
        robot.reset_encoders();
        robot.forward(-61,0.5);
        robot.wait_motors();
        robot.stop();
        if(direction != "straight"){
            parking(robot,direction);
        }
        
            
    }
    
     public void parking(StimDC robot,String dir){
            robot.reset_encoders();
            robot.lateral(60,0.5,dir);
            robot.wait_motors();
            robot.stop();
            
        }

   
}