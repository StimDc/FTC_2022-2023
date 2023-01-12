/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//the conversion has a 3% error for every 100 cm the robot runs for 103 cm



@Autonomous(name="TestareEncodere2022", group="Linear Opmode")
//@Disabled
public class Autonomiesemiencoder extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor front_left=null;
    private DcMotor front_right=null;
    private DcMotor back_left=null;
    private DcMotor back_right=null;
    private DcMotor slider=null;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double TICKS_PER_REV = 537.7d;
    private static final double PI = 3.14159265d;
    //private static final float GEAR_REDUCTION = 2.0f;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795d;
    // private static final double COUNTS_PER_INCH = (TICKS_PER_REV* GEAR_REDUCTION)/(WHEEL_CIRCUMFERENCE  * PI);
    private static float distanta =0;
    private static float pozitie=0;
    
    private static final double const_lateral_mov = 1.136363636363d;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Test");
        telemetry.update();

        //Links the virtual objets to the real motors
        front_left=hardwareMap.get(DcMotor.class, "FL");
        front_right=hardwareMap.get(DcMotor.class, "FR");
        back_left=hardwareMap.get(DcMotor.class, "BL");
        back_right=hardwareMap.get(DcMotor.class, "BR");



        //sets motors direction  (depends on how are the physical motors fixated)
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        //slider.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            run_using_encoders();
            reset_encoders();


           
            
            //left_right(30,0.3,"right");
            left_right(30,0.3, "left");
            while(front_left.isBusy()){

            }



            Stop();

            sleep(30000); //after execution, the program will wait until the times end so it doesnt loop
        }

    }

    //sets motors to run with encoders
    public void run_using_encoders(){
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //prepares the motors to run to position
    public void run_to_position(){
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //resets encoders for setting new distance
    public void reset_encoders(){
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //function to convert the distance in centimeters in inches

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
    public void left_right(double distance, double power, String dir){
        //double dist_loss_const = 1;
        //set_target_position(distance);
        
        distance = distance * const_lateral_mov;
        if(dir == "left"){
            distance  = -distance;
        }

        else if(dir == "right"){
            distance  = distance;
        }
        
        front_left.setTargetPosition(calculate_distance(distance));
        back_left.setTargetPosition(calculate_distance(-distance));
        front_right.setTargetPosition(calculate_distance(-distance));
        back_right.setTargetPosition(calculate_distance(distance));
        
        run_to_position();
        
        
        
        set_power(power);

    }
    //takes degrees power and direction as parameters
    // the program will convert degrees to distance and the robot will turn left or right
    public void rotate(double degrees, double power, String dir){
        double deg_to_dis = 1;

        double distance = 0;
        //set_target_position(distance);
        run_to_position();

        if(dir == "right"){
            power = power;
        }

        else if(dir == "left"){
            power = -power;
        }
        back_right.setPower(power);
        back_left.setPower(-power);
        front_right.setPower(power);
        back_left.setPower(-power);
    }

    //
    public void forward(double distance,double power){

        
        front_left.setTargetPosition(calculate_distance(distance));
        back_left.setTargetPosition(calculate_distance(distance));
        front_right.setTargetPosition(calculate_distance(distance));
        back_right.setTargetPosition(calculate_distance(distance));

        run_to_position();
        set_power(power);

        
        
    }
    
    public void set_power(double power){
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);
    }
    ///function
    public void Stop () {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}

