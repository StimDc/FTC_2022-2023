package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="TestareEncodere" , group="Linear Opmode")
//@Disabled
public class Autonomousceva extends LinearOpMode {

    public DcMotor LB;
    public DcMotor RB;
    public DcMotor LF;
    public DcMotor RF;
    public float TICKS = 537.7f;
    public float WHEEL = 11.811f;
    public float GEAR = 2.0f;
    public float COUNT = (TICKS * GEAR ) / (WHEEL * 3.1415f);

    @Override
    public void runOpMode() throws InterruptedException {
        LB = hardwareMap.dcMotor.get("BL");
        RB = hardwareMap.dcMotor.get("BR");
        LF = hardwareMap.dcMotor.get("FL");
        RF = hardwareMap.dcMotor.get("FR");

        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        usingEncoders();
        resetEncoders();

        float distance;
        waitForStart();
        distance = (convertCM(30)/WHEEL) *TICKS;
        distance = -distance; 
        RF.setTargetPosition((int)distance);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setPower(0.5);
        while(LF.isBusy()){
            
        }


    }
    public void usingEncoders(){

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoders(){
        //RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public float convertCM(double cm){
        return (float)cm/2.54f;
    }
}