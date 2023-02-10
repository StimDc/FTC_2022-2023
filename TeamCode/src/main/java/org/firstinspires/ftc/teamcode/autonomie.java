package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class autonomie extends StimDC{

    autonomie(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, DcMotor rotatoryBase, DcMotor slider, Servo c1, Servo c2) {
        super(frontLeft, frontRight, backLeft, backRight, rotatoryBase, slider, c1, c2);
    }

    //preloads the cone and move the slider onwards

    public void initSlider(){
        this.grabCone();

        this.sliderUp(12,this.getSliderPower());
        this.waitSlider();

        this.resetEncodersSlider();
        this.rotateRotatoryBase(180,this.getRotBasePower(),"right");
        this.waitRotatoryBase();
        this.resetEncodersRotatoryBase();
        this.stopRotatoryBase();
    }
    public void firstConeBigPole(String dir){
        String dir2;
        if(dir.equals("right")){
            dir2 = "left";
        }
        else{
            dir2 = "right";
        }
        this.resetEncodersWheelMotors();
        this.forward(123,this.getWheelPower());
        this.waitWheelMotors();
        this.stopWheelMotors();
        /*
        sleep(200);
        this.resetEncodersWheelMotors();
        this.forward(-27,this.getWheelPower());
        this.waitWheelMotors();
        this.stopWheelMotors();

         */

        this.sliderUp(75,this.getWheelPower());
        this.waitSlider();

        this.resetEncodersWheelMotors();
        if(dir.equals("right")){
            this.lateral(18,this.getWheelPower(),dir2);
        }
        else{
            this.lateral(17,this.getWheelPower(),dir2);
        }

        this.waitWheelMotors();
        this.stopWheelMotors();

        this.resetEncodersRotatoryBase();
        if(dir.equals("right")) {
            this.rotateRotatoryBase(65, this.getRotBasePower(), dir2);
        }
        else{
            this.rotateRotatoryBase(55, this.getRotBasePower(), dir2);

        }
        this.waitRotatoryBase();
        this.stopRotatoryBase();

        sleep(500);

        this.resetEncodersSlider();
        this.sliderDown(2,this.getSliderPower());
        this.waitSlider();

        this.releaseCone();

        this.resetEncodersRotatoryBase();
        if(dir.equals("right")){
            this.rotateRotatoryBase(62,this.getRotBasePower(),dir);
        }
        else{
            this.rotateRotatoryBase(55,this.getRotBasePower(),dir);
        }
        this.waitRotatoryBase();
        this.stopRotatoryBase();


        this.grabCone();

        this.runUsingEncodersSlider();


        this.stopSlider();

        this.resetEncodersRotatoryBase();
        if(dir == "right"){
            this.rotateRotatoryBase(90,this.getRotBasePower(),"right");
        }
        else{
            this.rotateRotatoryBase(90,this.getRotBasePower(),"left");
        }
        this.waitRotatoryBase();
        this.stopRotatoryBase();

    }

    void parking(String dir,String dir2){
        this.resetEncodersWheelMotors();
        if(dir2.equals("right")){
            if(dir.equals("left")){

                this.lateral(50,this.getWheelPower(),"left");

            }
            else if(dir.equals("straight")){
                this.lateral(25,this.getWheelPower(),"right");
            }
            else{
                this.lateral(70,this.getWheelPower(),"right");
            }
        }
        else{
            if(dir.equals("left")){

                this.lateral(70,this.getWheelPower(),"left");

            }
            else if(dir.equals("straight")){
                this.lateral(25,this.getWheelPower(),"left");
            }
            else{
                this.lateral(50,this.getWheelPower(),"right");
            }
        }
        this.waitWheelMotors();
        this.stopWheelMotors();




    }




}
