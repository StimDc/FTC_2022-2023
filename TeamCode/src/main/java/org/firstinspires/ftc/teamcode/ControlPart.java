package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="ControlPart")
public class ControlPart extends OpMode {

    DcMotor frontLeft,frontRight, backLeft, backRight;
    double drivePower;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {

        if(gamepad1.left_stick_y !=0){
            powerWheelMotors(gamepad1.left_stick_y, -1,-1,-1,-1);
        }

        if(gamepad1.right_stick_x !=0){
            //powerWheelMotors(gamepad1.right_stick_x,-1,1,-1,1);
            frontRight.setPower(-gamepad1.right_stick_x);
            frontLeft.setPower(gamepad1.right_stick_x);
            backRight.setPower(-gamepad1.right_stick_x);
            backLeft.setPower(gamepad1.right_stick_x);
        }

        if(gamepad1.left_trigger >0){
            frontRight.setPower(-gamepad1.left_trigger);
            frontLeft.setPower(gamepad1.left_trigger);
            backRight.setPower(gamepad1.left_trigger);
            backLeft.setPower(-gamepad1.left_trigger);

        }

        if(gamepad1.right_trigger >0){
            frontRight.setPower(gamepad1.right_trigger);
            frontLeft.setPower(-gamepad1.right_trigger);
            backRight.setPower(-gamepad1.right_trigger);
            backLeft.setPower(gamepad1.right_trigger);

        }


        debug(gamepad1);
        stop();


    }
    public void powerWheelMotors(float trigger, int sign1, int sign2, int sign3, int sign4){
        frontRight.setPower(sign1 *trigger);
        frontLeft.setPower(sign2 * trigger);
        backRight.setPower(sign3* trigger);
        backLeft.setPower(sign4* trigger);
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
    }
}
