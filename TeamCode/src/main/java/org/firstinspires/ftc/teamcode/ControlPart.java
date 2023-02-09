package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ControlPart")
public class ControlPart extends OpMode {

    String dpadDirection;
    char buttonLetter;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;

    private static final double SLIDER_WHEEL =PI * 0.440944882d;

    DcMotor frontLeft,frontRight, backLeft, backRight;
    DcMotor rotatoryBase,slider;
    Servo c1,c2;
    double sliderPower = 0.5;
    double heightSlider = 0;
    double bigPole = 84;
    double mediumPole = 60;
    double smallPole = 35;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");

        rotatoryBase = hardwareMap.get(DcMotor.class, "RB");
        slider = hardwareMap.get(DcMotor.class, "SL");

        c1 =hardwareMap.get(Servo.class, "C1");
        c2 = hardwareMap.get(Servo.class, "C2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rotatoryBase.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setDirection(DcMotorSimple.Direction.FORWARD);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotatoryBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        heightSlider = 0;


    }

    @Override
    public void loop() {

        //Handling movement
        if(gamepad1.left_stick_y >0.4 || gamepad1.left_stick_y <-0.4){
            powerWheelMotors(gamepad1.left_stick_y, -1,-1,-1,-1);
        }

        if(gamepad1.right_stick_x !=0){
            powerWheelMotors(gamepad1.right_stick_x,-1,1,-1,1);
        }

        if(gamepad1.left_trigger >0){
            powerWheelMotors(gamepad1.left_trigger,1,-1,-1,1);

        }

        if(gamepad1.right_trigger >0){
            powerWheelMotors(gamepad1.right_trigger,-1,1,1,-1);
        }

        if(gamepad2.right_stick_x !=0){
            //rotatoryBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rotatoryBase.setPower( -0.5*gamepad2.right_stick_x);
        }




        if(gamepad2.x){

            c1.setPosition(0);
            c2.setPosition(0.4);
        }
        if(gamepad2.y){
            c1.setPosition(0.4);
            c2.setPosition(0);
        }

        if(gamepad2.dpad_up){
            sliderMove(bigPole);
        }
        if(gamepad2.dpad_left){
            sliderMove(mediumPole);
        }
        if(gamepad2.dpad_down){
            sliderMove(smallPole);
        }
        if(gamepad2.dpad_right){
            sliderMove(8);
        }


        if(gamepad2.left_trigger !=0){
            frontRight.setPower(gamepad2.left_trigger);
            backRight.setPower(gamepad2.left_trigger);
        }
        if(gamepad2.right_trigger !=0){
            frontLeft.setPower(gamepad2.right_trigger);
            frontRight.setPower(-gamepad2.right_trigger);
        }


        debug(gamepad2);
        sleep(20);
        stop();


    }
    public void powerWheelMotors(float trip, int sign1, int sign2, int sign3, int sign4){

        frontRight.setPower(sign1 * trip);
        frontLeft.setPower(sign2 * trip);
        backRight.setPower(sign3* trip);
        backLeft.setPower(sign4* trip);
    }
    public void debug(Gamepad gamepad){

        telemetry.addLine("Gamepad left stick x: " + gamepad.left_stick_x);
        telemetry.addLine("Gamepad left stick y: " + gamepad.left_stick_y);
        telemetry.addLine("Gamepad right stick x: " + gamepad.right_stick_x);
        telemetry.addLine("Gamepad right stick y: " + gamepad.right_stick_y);
        telemetry.addLine("Gamepad left trigger: " + gamepad.left_trigger);
        telemetry.addLine("Gamepad right trigger: " + gamepad.right_trigger);

        if(gamepad.dpad_up){
            dpadDirection = "UP";
        }
        else if(gamepad.dpad_right){
            dpadDirection = "RIGHT";
        }
        else if(gamepad.dpad_down){
            dpadDirection = "DOWN";
        }
        else if(gamepad.dpad_left){
            dpadDirection = "LEFT";
        }

        telemetry.addLine("Gamepad dpad: " + dpadDirection);

        if(gamepad.a){
            buttonLetter = 'A';
        }
        else if(gamepad.b){
            buttonLetter = 'B';
        }
        else if(gamepad.x){
            buttonLetter = 'X';
        }
        else if(gamepad.y){
            buttonLetter = 'Y';
        }

        telemetry.addLine("Gamepad buttons: " + buttonLetter);
        telemetry.update();
    }
    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        rotatoryBase.setPower(0);


    }

    public void sliderMove(double poleHeight){
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(heightSlider != poleHeight) {
            setSlider(DcMotorSimple.Direction.REVERSE, poleHeight,1);
        }
        else{
            slider.setPower(0);
            heightSlider = 0;
        }
        sleep(1000);
    }

    public void setSlider(DcMotorSimple.Direction direction, double poleHeight, int sign){
        slider.setPower(0);
        slider.setDirection(direction);
        int diss = calculateDistance( sign *poleHeight - heightSlider, SLIDER_WHEEL,GO_TICKS_PER_REV);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(diss);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        heightSlider = poleHeight;
        slider.setPower(sliderPower);

    }
    public int calculateDistance(double dis, double wheelCircumference, double ticks){
        dis = dis / 2.54;
        int diss = (int) ((dis / wheelCircumference) * ticks);
        diss /=4;
        diss *=0.9090909d;
        return diss;
    }


}
