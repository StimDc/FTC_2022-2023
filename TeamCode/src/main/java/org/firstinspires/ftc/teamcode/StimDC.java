package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.teamcode.Stim;
import com.qualcomm.robotcore.hardware.DcMotor;

//Class to handle robot movements to reuse in other files
public class StimDC {
    private static final double GO_TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;

    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795d;
    private static final double CONST_LATERAL_MOVEMENT = 1.136363636363d;
    private static final double SLIDER_WHEEL =PI * 0.440944882d;
    private static final double ROTATORY_BASE = PI *5.90551181d;
    private static final double REV_TICKS_PER_REV = 288d;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor slider ;
    private DcMotor rotatory_base;
    private DcMotor arm;
    private Servo c1 ;
    private Servo c2;
    public double camera_fx = 578.272;
    public double camera_fy = 578.272;
    public double camera_cx = 402.145;
    public double camera_cy = 221.506;



    StimDC(DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right,DcMotor rotatory_base, DcMotor slider,DcMotor arm,Servo c1, Servo c2) {

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
        rotatory_base.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
        this.slider = slider;
        this.rotatory_base = rotatory_base;
        this.arm = arm;
        this.c1 = c1;
        this.c2 = c2;



    }
    /*
   StimDC(){
        init_slider(c1, c2);
       init_wheel_motor(front_left, front_right, back_left, back_right);
   }

    
    public void init_wheel_motors(DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right){
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
    }

    public void init_slider(DcMotor slider, Servo c1, Servo c2){
        slider.setDirection(DcMotor.Direction.FORWARD);

        this.slider = slider;
        this.c1 = c1;
        this.c2 = c2;
    }
    */
    //sets motors to run with encoders
    public void run_using_encoders_wheel_motors(){
        this.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run_using_encoders_slider(){
        this.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void run_using_encoders(){
        run_using_encoders_wheel_motors();
        run_using_encoders_slider();
        this.rotatory_base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //prepares the motors to run to position
    public void run_to_position(){
        this.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    //resets encoders for setting new distance

    public void reset_encoders_wheel_motors(){
        this.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void reset_encoders_slider(){
        this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void reset_encoders(){
        reset_encoders_wheel_motors();
        reset_encoders_slider();
        //this.rotatory_base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    //function to convert the distance in centimeters in inches
    public double cm_to_inch(double cm){
        return (cm/2.54);
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
        if(dir.equals("left")){
            distance  = -distance;
        }

        else if(dir.equals("right")){
            distance = distance;
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
        if(dir.equals("right")){
            degrees = degrees;

        }
        else if (dir.equals("left")){

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

        if(dir.equals("right")){
            distance  = distance;
        }

        else if(dir.equals("left")){
            distance = -distance;
        }

        this.front_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.front_right.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_right.setTargetPosition(calculate_distance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        set_power(power);
    }

    public void forward(double distance,double power){

        distance = distance * 0.934579439252d;
        this.front_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_left.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.front_right.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.back_right.setTargetPosition(calculate_distance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        run_to_position();
        set_power(power);



    }

    public void grab_cone(){
        this.c1.setPosition(0.4);
        this.c2.setPosition(0);
    }
    public void release_cone(){
        this.c1.setPosition(0);
        this.c2.setPosition(0.4);
    }

    public void sliderup(double distance, double power){
        //distance = distance;
        this.slider.setTargetPosition(calculate_distance(distance,SLIDER_WHEEL,GO_TICKS_PER_REV));
        this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slider.setPower(power);

    }

    public void sliderdown (double distance, double power){
        distance = -distance;
        this.slider.setTargetPosition(calculate_distance(distance,SLIDER_WHEEL,GO_TICKS_PER_REV));
        this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slider.setPower(power);
    }


    public void armextend(double distance, double power){
        this.arm.setTargetPosition(calculate_distance(distance,0,GO_TICKS_PER_REV));
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(power);
    }
    public void armretract(double distance ,double power){
        distance = -distance;
        this.arm.setTargetPosition(calculate_distance(distance,0,GO_TICKS_PER_REV));
        this.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.arm.setPower(power);
    }
    public void grab_claw(){
        this.c1.setPosition(0.4);
        this.c2.setPosition(0);
    }

    public void release_claw(){
        this.c1.setPosition(0);
        this.c2.setPosition(0.4);
    }
    public void set_power(double power){
        this.front_left.setPower(power);
        this.front_right.setPower(power);
        this.back_left.setPower(power);
        this.back_right.setPower(power);


    }
    public void stop_wheel_motors(){
        this.front_right.setPower(0);
        this.front_left.setPower(0);
        this.back_right.setPower(0);
        this.back_left.setPower(0);

    }
    public void stop_slider(){
        this.slider.setPower(0);
    }
    public void stop () {
        stop_wheel_motors();
        stop_slider();
        this.rotatory_base.setPower(0);
        this.arm.setPower(0);
    }
    public void wait_wheel_motors(){
        //telemetry.addLine("waiting wheel motors");
        //telemetry.update();
        while(this.front_left.isBusy() && this.front_right.isBusy() && this.back_left.isBusy() && this.back_right.isBusy()){

        }
    }
    public void wait_slider(){
        //telemetry.addLine("waiting slider motor");
        //telemetry.update();
        while(this.slider.isBusy()){

        }
    }
    public void wait_motors(){
        //telemetry.addLine("waiting ALL motors");
        //telemetry.update();
        while((this.front_left.isBusy() && this.front_right.isBusy() && this.back_left.isBusy() && this.back_right.isBusy() ) || this.rotatory_base.isBusy() || this.slider.isBusy()){

        }
    }

}