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




@Autonomous(name="Dreapta mare fara camera", group="Linear Opmode")
//@Disabled
public class DreaptaMareFaraCamera extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor front_left=null;
    private DcMotor front_right=null;
    private DcMotor back_left=null;
    private DcMotor back_right=null;
    private DcMotor rotatory_base=null;
    private ElapsedTime runtime = new ElapsedTime();

    
    
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Test");
        telemetry.update();

        //Links the virtual objets to the real motors
        front_left=hardwareMap.get(DcMotor.class, "FL");
        front_right=hardwareMap.get(DcMotor.class, "FR");
        back_left=hardwareMap.get(DcMotor.class, "BL");
        back_right=hardwareMap.get(DcMotor.class, "BR");
        rotatory_base = hardwareMap.get(DcMotor.class, "RB");

        //Initialises the robot
        StimDC robot = new StimDC(front_left,front_right,back_left,back_right, rotatory_base);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.run_using_encoders();
            
            robot.reset_encoders();
            robot.forward(130,0.5);
            robot.wait_motors();
            robot.stop();
            
            robot.reset_encoders();
            robot.lateral(30,0.5,"right");
            robot.wait_motors();
            robot.stop();
            
            robot.run_using_encoders();
            robot.reset_encoders();
            robot.slider_base_rotate(65,0.9,"left");
            robot.wait_motors();
            robot.stop();
            sleep(1000);
            for(int i = 1;i<=5;i++){
               robot.reset_encoders();
                robot.slider_base_rotate(170,0.9,"right");
                robot.wait_motors();
                robot.stop();
                sleep(1000); 
                
                robot.reset_encoders();
                robot.slider_base_rotate(170,0.9,"left");
                robot.wait_motors();
                robot.stop();
                sleep(1000);
            }
            
            robot.run_using_encoders();
            robot.reset_encoders();
            robot.lateral(30,0.5,"left");
            robot.wait_motors();
            robot.stop();
            
            robot.reset_encoders();
            robot.forward(-125,0.5);
            robot.wait_motors();
            robot.stop();
            
            robot.run_using_encoders();
            robot.reset_encoders();
            robot.lateral(80,0.5,"left");
            robot.wait_motors();
            robot.stop();
            
            sleep(30000);
            //after execution, the program will wait until the times end so it doesnt loop
        }

    }

}





