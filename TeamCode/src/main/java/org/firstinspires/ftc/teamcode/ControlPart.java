package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ControlPart")
public class ControlPart extends OpMode {


    double contor = 0;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795d;
    private static final double CONST_LATERAL_MOVEMENT = 1.136363636363d;
    private static final double SLIDER_WHEEL =PI * 0.440944882d;
    private static final double ROTATORY_BASE = PI *5.90551181d;
    DcMotor frontLeft,frontRight, backLeft, backRight;
    DcMotor rotatoryBase,slider;
    Servo c1,c2;
    double drivePower;
    double heightSlider = 0;
    double bigPole = 70;
    double mediumPole = 60;
    double smallPole = 1;
    static int disss;

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
        //slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        //Handling movement
        if(gamepad1.left_stick_y !=0){
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

        if(gamepad2.left_stick_x !=0){
            rotatoryBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rotatoryBase.setPower( -0.6*gamepad2.left_stick_x);
        }
        /*
        if(gamepad2.right_stick_y!=0){
            //slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slider.setPower(0.6*gamepad2.right_stick_y);
        }

         */

        if(gamepad2.x){
            c1.setPosition(0);
            c2.setPosition(0.4);
        }
        if(gamepad2.y){
            c1.setPosition(0.4);
            c2.setPosition(0);
        }

        if(gamepad2.b){
            if(heightSlider != bigPole) {
                double dis = bigPole - heightSlider;
                dis = dis / 2.54;

                int diss = (int) ((dis / SLIDER_WHEEL) * GO_TICKS_PER_REV);
                diss /=4;
                diss *=0.9090909d;
                telemetry.addData("dis", diss);
                telemetry.update();
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setTargetPosition(diss);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                heightSlider = bigPole;
                slider.setPower(0.1);

            }
        }
        telemetry.addData("contor",contor);
        //telemetry.update();
        if(gamepad2.a){
            if(heightSlider != mediumPole) {
                double dis = mediumPole - heightSlider;
                dis = dis / 2.54;

                int diss = (int) ((dis / SLIDER_WHEEL) * GO_TICKS_PER_REV);
                diss /=4;
                diss *=0.9090909d;
                telemetry.addData("dis", diss);
                telemetry.update();
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setTargetPosition(diss);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                heightSlider = mediumPole;
                slider.setPower(0.1);

            }

        }
        telemetry.addData("distance",heightSlider);
        telemetry.update();



        if(gamepad2.left_trigger !=0){
            frontRight.setPower(gamepad2.left_trigger);
            backRight.setPower(gamepad2.left_trigger);
        }
        if(gamepad2.right_trigger !=0){
            frontLeft.setPower(gamepad2.right_trigger);
            frontRight.setPower(-gamepad2.right_trigger);
        }






        //debug(gamepad1);
        stop();


    }
    public void powerWheelMotors(float trip, int sign1, int sign2, int sign3, int sign4){
        frontRight.setPower(sign1 *trip);
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
        telemetry.update();
    }
    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        rotatoryBase.setPower(0);
        //slider.setPower(0);

        rotatoryBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public int calculateDistance(double dis, double wheelCircumference, double ticks){
        dis = dis / 2.54;
        int diss = (int) ((dis / SLIDER_WHEEL) * GO_TICKS_PER_REV);
        diss /=4;
        diss *=0.9090909d;
        return diss;
    }


    public void sliderMove(double poleHeight){
        if(heightSlider != poleHeight) {
            int diss = calculateDistance(poleHeight - heightSlider,SLIDER_WHEEL,GO_TICKS_PER_REV);
            telemetry.addData("dis", diss);
            telemetry.update();
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setTargetPosition(diss);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            heightSlider = poleHeight;
            slider.setPower(0.1);

        }
    }
}
