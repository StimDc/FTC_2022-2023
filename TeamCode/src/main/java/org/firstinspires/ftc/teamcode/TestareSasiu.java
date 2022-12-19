package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="TestareSasiu", group="Linear Opmode")
//@Disabled
public class TestareSasiu extends OpMode {

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;

    double drivePower;
    double sliderPower;
    double rotating_devicePower;
    double brat_position = 0;
    double claw_position = 0;
    double vacuum_Power;


    @Override
    public void init() {
        front_left = hardwareMap.dcMotor.get("FL");
        front_right = hardwareMap.dcMotor.get("FR");
        back_left = hardwareMap.dcMotor.get("BL");
        back_right = hardwareMap.dcMotor.get("BR");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        
        

    }

    //primul gamepad o sa fie pentru miscarea robotului inainte, inapoi, stanga, dreapta si rotire
    //al doilea gamepad o sa fie pentru lansarea discurilor si pentru brat
    @Override
    public void loop() {

        //o sa avem doua moduri pentru viteza pe butoanele a si b
        if (gamepad1.a) {
            drivePower = 0.3;
        }else if (gamepad1.b) {
            drivePower = 0.7;
        }else stop ();
        //miscarea se va face din cele doua manete de pe primul gamepad
    /*
        if (gamepad1.left_stick_x > 0.2 && gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_y < 0 && gamepad2.right_trigger > 0) {
            front_left.setPower(drivePower);
            front_right.setPower(drivePower);
            back_left.setPower(drivePower);
            back_right.setPower(drivePower);
        } else if (gamepad1.left_stick_x > 0.2 && gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_y < 0 && gamepad2.left_trigger > 0) {
            front_left.setPower(drivePower);
            front_right.setPower(drivePower);
            back_left.setPower(drivePower);
            back_right.setPower(drivePower);
        } else if (gamepad1.left_stick_x > 0.2 && gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_y > 0 && gamepad2.right_trigger > 0) {
            front_left.setPower(-drivePower);
            front_right.setPower(-drivePower);
            back_left.setPower(-drivePower);
            back_right.setPower(-drivePower);
        } else if (gamepad1.left_stick_x > 0.2 && gamepad1.left_stick_x < 0.2 && gamepad1.left_stick_y > 0 && gamepad2.left_trigger > 0) {
            front_left.setPower(-drivePower);
            front_right.setPower(-drivePower);
            back_left.setPower(-drivePower);
            back_right.setPower(-drivePower);
        } else stop ();
    */
        if ((gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_x < 0.1) && gamepad1.left_stick_y < -0.1){ //fata
            front_left.setPower(-drivePower);
            front_right.setPower(-drivePower);
            back_left.setPower(drivePower);
            back_right.setPower(drivePower);
        } else if ((gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_x < 0.1) && gamepad1.left_stick_y > 0.1){ //spate
            front_left.setPower(drivePower);
            front_right.setPower(drivePower);
            back_left.setPower(-drivePower);
            back_right.setPower(-drivePower);
        }else stop();
        
        ///stanga-dreapta
        if (gamepad1.left_bumper){
            front_left.setPower(-drivePower);
            front_right.setPower(drivePower);
            back_left.setPower(drivePower);
            back_right.setPower(-drivePower);
        } else if (gamepad1.right_bumper) {
            front_left.setPower(drivePower);
            front_right.setPower(-drivePower);
            back_left.setPower(-drivePower);
            back_right.setPower(drivePower);
        }else stop();
        /*if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0){
            stop();
        }*/

        //maneta dreapta de pe primul gamepad o sa fie pentru a roti robotul spre stanga sau spre dreapta
        if(gamepad1.right_stick_x < -0.5 && gamepad1.right_stick_y == 0){
            front_left.setPower(-drivePower);
            front_right.setPower(drivePower);
            back_left.setPower(-drivePower);
            back_right.setPower(drivePower);
        }else if(gamepad1.right_stick_x > 0.5 && gamepad1.right_stick_y == 0){
            front_left.setPower(drivePower);
            front_right.setPower(-drivePower);
            back_left.setPower(drivePower);
            back_right.setPower(-drivePower);
        } else stop();
    }   

    @Override
    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
        
    }