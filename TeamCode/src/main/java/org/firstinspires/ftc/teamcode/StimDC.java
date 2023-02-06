package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Servo;
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
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor slider ;
    private DcMotor rotatoryBase;
    private DcMotor arm;
    private Servo c1 ;
    private Servo c2;
    public double camera_fx = 578.272;
    public double camera_fy = 578.272;
    public double camera_cx = 402.145;
    public double camera_cy = 221.506;

    private double wheelPower = 0.5;
    private double sliderPower = 0.5;
    private double rotBasePower = 0.5;


    StimDC(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,DcMotor rotatoryBase, DcMotor slider,Servo c1, Servo c2) {

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.REVERSE);
        rotatoryBase.setDirection(DcMotor.Direction.REVERSE);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setDirection(DcMotor.Direction.FORWARD);

        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.slider = slider;
        this.rotatoryBase = rotatoryBase;
        //this.arm = arm;
        this.c1 = c1;
        this.c2 = c2;



    }
    
    //sets motors to run with encoders
    public void runUsingEncodersWheelMotors(){
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runUsingEncodersRotatoryBase(){
        this.rotatoryBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runUsingEncodersSlider(){
        this.slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runUsingEncoders(){
        runUsingEncodersWheelMotors();
        runUsingEncodersSlider();
        runUsingEncodersRotatoryBase();

    }

    //prepares the motors to run to position
    public void runToPosition(){
        this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    //resets encoders for setting new distance

    public void resetEncodersWheelMotors(){
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetEncodersSlider(){
        this.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetEncodersRotatoryBase(){
        this.rotatoryBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetEncoders(){
        resetEncodersWheelMotors();
        resetEncodersSlider();
        resetEncodersRotatoryBase();


    }

    ///converts physical distance into ticks so that the robot can understand
    public int  calculateDistance(double distance,double wheel_diameter, double ticks){
        distance = distance / 2.54;
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

        this.frontLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backLeft.setTargetPosition(calculateDistance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.frontRight.setTargetPosition(calculateDistance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        runToPosition();
        setPower(power);

    }



    public void rotateDiagonal(double distance, double power , String dir){
        if(dir.equals("right")){
            this.frontLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
            this.backLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

            this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.frontLeft.setPower(power);
            this.backLeft.setPower(power);
        }
        else{
            this.frontRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
            this.backRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

            this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            this.frontRight.setPower(power);
            this.backRight.setPower(power);
        }

    }

    public void rotateSpecial(double distance, double power, String dir){
        if(dir.equals("right")){
            distance = distance;
        }
        else{
            distance = -distance;
        }

        this.frontRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.frontLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
    }
    //rotates the rotatory base for the slider
    public void rotateRotatoryBase(double degrees,double power,String dir){
        double dis_for_right_angle = 11.4;

        this.rotatoryBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(dir.equals("right")){
            degrees = degrees;

        }
        else if (dir.equals("left")){

            degrees = -degrees;

        }
        double distance = (degrees/90) * dis_for_right_angle;
        this.rotatoryBase.setTargetPosition(calculateDistance(distance,ROTATORY_BASE,REV_TICKS_PER_REV));

        this.rotatoryBase.setPower(power);

    }
    //function for the rotational movement, takes degrees and converts into distance
    public void rotate(double degrees, double power, String dir){

        //constant used for distance, for every 90 degree angle turn, the robot needs 55 cm
        double  dis_for_right_angle =54.5;

        double distance = (degrees/90) * dis_for_right_angle;

        runToPosition();

        if(dir.equals("right")){
            distance  = distance;
        }

        else if(dir.equals("left")){
            distance = -distance;
        }

        this.frontLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.frontRight.setTargetPosition(calculateDistance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backRight.setTargetPosition(calculateDistance(-distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        //set_power(power);
        fluidPower(power);
    }



    public void forward(double distance,double power){

        distance = distance * 0.934579439252d;
        this.frontLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backLeft.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.frontRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));
        this.backRight.setTargetPosition(calculateDistance(distance,WHEEL_CIRCUMFERENCE,GO_TICKS_PER_REV));

        runToPosition();
        //set_power(power);
        fluidPower(power);
        

        



    }

    private void fluidPower(double power){
        for(double i =0.0;i<=power;i+=0.1){
            setPower(i);
            sleep(20);
        }
    }

    private void fluidPowerRotBase(double power){
        for(double i = 0.0;i<=power;i++){
            this.rotatoryBase.setPower(i);
        }
    }

    public void grabCone(){
        this.c1.setPosition(0.4);
        this.c2.setPosition(0);
    }
    public void releaseCone(){
        this.c1.setPosition(0);
        this.c2.setPosition(0.4);
    }

    public void sliderUp(double distance, double power){
        //distance = distance;
        this.slider.setTargetPosition(calculateDistance(distance,SLIDER_WHEEL,GO_TICKS_PER_REV));
        this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slider.setPower(power);

    }

    public void sliderDown (double distance, double power){
        distance = -distance;
        this.slider.setTargetPosition(calculateDistance(distance,SLIDER_WHEEL,GO_TICKS_PER_REV));
        this.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slider.setPower(power);
    }




    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);


    }
    public void stopWheelMotors(){
        this.frontRight.setPower(0);
        this.frontLeft.setPower(0);
        this.backRight.setPower(0);
        this.backLeft.setPower(0);

    }
    public void stopRotatoryBase(){
        this.rotatoryBase.setPower(0);
    }
    public void stopSlider(){
        this.slider.setPower(0);

    }
    public void stop () {
        stopWheelMotors();
        stopSlider();
        this.rotatoryBase.setPower(0);
        this.arm.setPower(0);
    }


    public void waitRotatoryBase(){
        while(this.rotatoryBase.isBusy()){

        }
    }

    public void waitSlider(){
        //telemetry.addLine("waiting slider motor");
        //telemetry.update();
        while(this.slider.isBusy() ){

        }
    }
    public void waitMotors(){
        //telemetry.addLine("waiting ALL motors");
        //telemetry.update();
        while((this.frontLeft.isBusy() && this.frontRight.isBusy() && this.backLeft.isBusy() && this.backRight.isBusy() ) || this.rotatoryBase.isBusy() || this.slider.isBusy()){

        }
    }
    public void setWheelPower(double wheelPower){
        this.wheelPower  = wheelPower;
    }
    public double getWheelPower(){
        return this.wheelPower;
    }

    public void setSliderPower(double sliderPower){
        this.sliderPower = sliderPower;
    }
    public double getSliderPower(){
        return this.sliderPower;
    }
    public void setRotBasePower(double rotBasePower){
        this.rotBasePower = rotBasePower;
    }
    
    public double getRotBasePower(){
        return this.rotBasePower;
    }




    /*
     * renaming  functions
     */

     public void waitWheelMotors(){
        while(this.frontLeft.isBusy() && this.frontRight.isBusy() && this.backLeft.isBusy() && this.backRight.isBusy()){
            
        }
     }




}

