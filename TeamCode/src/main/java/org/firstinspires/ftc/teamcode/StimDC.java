package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;



//Class to handle robot movements to reuse in other files
public class StimDC {
    private static final double GO_TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;

    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795d;
    private static final double CONST_LATERAL_MOVEMENT = 1.136363636363d;
    private static final double SLIDER_WHEEL = 0.440944882d;
    private static final double ROTATORY_BASE = 5.90551181d;
    private static final double REV_TICKS_PER_REV = 288d;
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    //private DcMotor slider = null;
    private DcMotor rotatory_base = null;
    public double camera_fx = 578.272;
    public double camera_fy = 578.272;
    public double camera_cx = 402.145;
    public double camera_cy = 221.506;

    StimDC(DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right,DcMotor rotatory_base) {
        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
        //this.slider = slider;
        this.rotatory_base = rotatory_base;

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        //slider.setDirection(DcMotor.Direction.FORWARD);
        rotatory_base.setDirection(DcMotor.Direction.REVERSE);

    }

    //sets motors to run with encoders
    public void run_using_encoders(){
        this.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rotatory_base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //prepares the motors to run to position
    public void run_to_position(){
        this.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    //resets encoders for setting new distance
    public void reset_encoders(){
        this.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rotatory_base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //function to convert the distance in centimeters in inches
    public double cm_to_inch(double cm){
        return (double)(cm/2.54);
    }
    ///converts physical distance into ticks so that the robot can understand
    public int  calculate_distance(double distance,double wheel_diameter, double ticks){
        distance = cm_to_inch(distance);

        return (int)((distance/wheel_diameter) * ticks);
    }

    //takes distance power and direction as parameters,
    //if direction is set to "left" the robot will move to left, if set to "right" the robot will move to right

    //dist_loss_const is a constant used for recalculating the distance because when doing the complex moves the robot will lose distance
    public void lateral(double distance, double power, String dir){


        distance = distance * CONST_LATERAL_MOVEMENT;
        if(dir == "left"){
            distance  = -distance;
        }

        else if(dir == "right"){
            distance  = distance;
        }

        this.front_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_left.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.front_right.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_right.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        run_to_position();



        set_power(power);

    }

    //rotates the rotatory base for the slider
    public void slider_base_rotate(double degrees,double power,String dir){
        double dis_for_right_angle = 3.75;

        this.rotatory_base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(dir == "right"){
            degrees = degrees;

        }
        else if (dir == "left"){

            degrees = -degrees;

        }
        double distance = (degrees/90) * dis_for_right_angle;
        this.rotatory_base.setTargetPosition(calculate_distance(distance,ROTATORY_BASE,REV_TICKS_PER_REV));

        this.rotatory_base.setPower(power);

    }
    //function for the rotational movement, takes degrees and converts into distance
    public void rotate(double degrees, double power, String dir){

        //constant used for distance, for every 90 degree angle turn, the robot needs 55 cm
        double  dis_for_right_angle =54.5;

        double distance = (degrees/90) * dis_for_right_angle;

        run_to_position();

        if(dir == "right"){
            distance  = distance;
        }

        else if(dir == "left"){
            distance = -distance;
        }

        this.front_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.front_right.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_right.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        set_power(power);
    }

    public void forward(double distance,double power){


        this.front_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.front_right.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_right.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        run_to_position();
        set_power(power);



    }
    /*
    public void sliderup(double distance, double power){
        this.slider.setTargetPosition(calculate_slider(distance));
        this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slider.setPower(power);

    }
    */

    public void set_power(double power){
        this.front_left.setPower(power);
        this.front_right.setPower(power);
        this.back_left.setPower(power);
        this.back_right.setPower(power);


    }

    public void stop () {
        this.front_left.setPower(0);
        this.front_right.setPower(0);
        this.back_left.setPower(0);
        this.back_right.setPower(0);
        //this.slider.setPower(0);
        this.rotatory_base.setPower(0);
    }

    public void wait_motors(){
        while((this.front_left.isBusy() && this.front_right.isBusy() && this.back_left.isBusy() && this.back_right.isBusy() ) || this.rotatory_base.isBusy()){

        }
    }
    /*
    public void tagToTelemetry(AprilTagDetection detection){
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f meters", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f meters", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f meters", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    
    */
}