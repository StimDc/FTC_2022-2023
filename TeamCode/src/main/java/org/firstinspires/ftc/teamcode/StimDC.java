package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

//Class to handle robot movements to reuse in other files
public class StimDC {
    private static final double TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;
    //private static final float GEAR_REDUCTION = 2.0f;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795d;
    private static final double const_lateral_mov = 1.136363636363d;

    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;



    StimDC(DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right) {
        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;


        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

    }

    //sets motors to run with encoders
    public void run_using_encoders(){
        this.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }
    //function to convert the distance in centimeters in inchesv
    public double cm_to_inch(double cm){
        return (double)(cm/2.54);
    }
    ///converts physical distance into ticks so that the robot can understand
    public int  calculate_distance(double distance){
        distance = cm_to_inch(distance);

        return (int)((distance/WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);
    }

    //takes distance power and direction as parameters,
    //if direction is set to "left" the robot will move to left, if set to "right" the robot will move to right


    //dist_loss_const is a constant used for recalculating the distance because when doing the complex moves the robot will lose distance
    public void lateral(double distance, double power, String dir){


        distance = distance * const_lateral_mov;
        if(dir == "left"){
            distance  = -distance;
        }

        else if(dir == "right"){
            distance  = distance;
        }

        this.front_left.setTargetPosition(calculate_distance(distance));
        this.back_left.setTargetPosition(calculate_distance(-distance));
        this.front_right.setTargetPosition(calculate_distance(-distance));
        this.back_right.setTargetPosition(calculate_distance(distance));

        run_to_position();



        set_power(power);

    }

    //takes degrees power and direction as parameters
    // the program will convert degrees to distance and the robot will turn left or right
    public void rotate(double degrees, double power, String dir){

        //constant used for distance, for every 90 degree angle turn, the robot needs 55 cm
        double  dis_for_right_angle = 54.5;

        double distance = (degrees/90) * dis_for_right_angle;

        run_to_position();

        if(dir == "right"){
            distance  = distance;
        }

        else if(dir == "left"){
            distance = -distance;
        }

        this.front_left.setTargetPosition(calculate_distance(distance));
        this.front_right.setTargetPosition(calculate_distance(-distance));
        this.back_left.setTargetPosition(calculate_distance(distance));
        this.back_right.setTargetPosition(calculate_distance(-distance));

        set_power(power);
    }

    public void forward(double distance,double power){


        this.front_left.setTargetPosition(calculate_distance(distance));
        this.back_left.setTargetPosition(calculate_distance(distance));
        this.front_right.setTargetPosition(calculate_distance(distance));
        this.back_right.setTargetPosition(calculate_distance(distance));

        run_to_position();
        set_power(power);



    }
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
    }
    
    public void wait_motors(){
        while(this.front_left.isBusy() && this.front_right.isBusy() && this.back_left.isBusy() && this.back_right.isBusy()){
            
        }
    }

}
