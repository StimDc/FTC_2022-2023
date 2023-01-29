package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

//import org.firstinspires.ftc.teamcode.Stim;
//Class to handle robot movements to reuse in other files
class KStimDC internal constructor(
    front_left: DcMotor,
    front_right: DcMotor,
    back_left: DcMotor,
    back_right: DcMotor,
    rotatory_base: DcMotor,
    slider: DcMotor,
    arm: DcMotor,
    c1: Servo,
    c2: Servo
) {
    private val front_left: DcMotor
    private val front_right: DcMotor
    private val back_left: DcMotor
    private val back_right: DcMotor
    private val slider: DcMotor
    private val rotatory_base: DcMotor
    private val arm: DcMotor
    private val c1: Servo
    private val c2: Servo
    var camera_fx = 578.272
    var camera_fy = 578.272
    var camera_cx = 402.145
    var camera_cy = 221.506
    var wheelPower = 0.5
    var sliderPower = 0.5
    var rotBasePOwer = 0.5
        private set

    init {
        front_left.direction = DcMotorSimple.Direction.REVERSE
        front_right.direction = DcMotorSimple.Direction.FORWARD
        back_left.direction = DcMotorSimple.Direction.REVERSE
        back_right.direction = DcMotorSimple.Direction.FORWARD
        slider.direction = DcMotorSimple.Direction.FORWARD
        rotatory_base.direction = DcMotorSimple.Direction.REVERSE
        arm.direction = DcMotorSimple.Direction.FORWARD
        this.front_left = front_left
        this.front_right = front_right
        this.back_left = back_left
        this.back_right = back_right
        this.slider = slider
        this.rotatory_base = rotatory_base
        this.arm = arm
        this.c1 = c1
        this.c2 = c2
    }

    /*
   StimDC(){
        init_slider(c1, c2);
       init_wheel_motor(front_left, front_right, back_left, back_right);
   }


    public void init_wheel_motors(DcMotor front_left, DcMotor front_right, DcMotor back_left, DcMotor back_right){
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
    }

    public void init_slider(DcMotor slider, Servo c1, Servo c2){
        slider.setDirection(DcMotor.Direction.FORWARD);

        this.slider = slider;
        this.c1 = c1;
        this.c2 = c2;
    }
    */
    //sets motors to run with encoders
    fun run_using_encoders_wheel_motors() {
        front_left.mode = DcMotor.RunMode.RUN_USING_ENCODER
        front_right.mode = DcMotor.RunMode.RUN_USING_ENCODER
        back_left.mode = DcMotor.RunMode.RUN_USING_ENCODER
        back_right.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun run_using_encoders_rotatory_slider() {
        rotatory_base.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun run_using_encoders_slider() {
        slider.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun run_using_encoders() {
        run_using_encoders_wheel_motors()
        run_using_encoders_slider()
        run_using_encoders_rotatory_slider()
        //this.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //prepares the motors to run to position
    fun run_to_position() {
        back_left.mode = DcMotor.RunMode.RUN_TO_POSITION
        back_right.mode = DcMotor.RunMode.RUN_TO_POSITION
        front_left.mode = DcMotor.RunMode.RUN_TO_POSITION
        front_right.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    //resets encoders for setting new distance
    fun reset_encoders_wheel_motors() {
        back_right.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        back_left.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        front_left.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        front_right.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun reset_encoders_slider() {
        slider.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun reset_encoders_rotatory_base() {
        rotatory_base.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun reset_encoders() {
        reset_encoders_wheel_motors()
        reset_encoders_slider()
        reset_encoders_rotatory_base()
        //this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //function to convert the distance in centimeters in inches
    fun cm_to_inch(cm: Double): Double {
        return cm / 2.54
    }

    ///converts physical distance into ticks so that the robot can understand
    fun calculate_distance( distance: Double, wheel_diameter: Double, ticks: Double): Int {
        var distance = cm_to_inch(distance)
        return (distance / wheel_diameter * ticks).toInt()
    }

    //takes distance power and direction as parameters,
    //if direction is set to "left" the robot will move to left, if set to "right" the robot will move to right
    //dist_loss_const is a constant used for recalculating the distance because when doing the complex moves the robot will lose distance
    fun lateral(distance: Double, power: Double, dir: String) {
        var distance = distance
        distance = distance * CONST_LATERAL_MOVEMENT
        if (dir == "left") {
            distance = -distance
        } else if (dir == "right") {
            distance = distance
        }
        front_left.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_left.targetPosition =
            calculate_distance(-distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        front_right.targetPosition =
            calculate_distance(-distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_right.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        run_to_position()
        set_power(power)
    }

    //rotates the rotatory base for the slider
    fun rotate_rotatory_base(degrees: Double, power: Double, dir: String) {
        var degrees = degrees
        val dis_for_right_angle = 11.4
        rotatory_base.mode = DcMotor.RunMode.RUN_TO_POSITION
        if (dir == "right") {
            degrees = degrees
        } else if (dir == "left") {
            degrees = -degrees
        }
        val distance = degrees / 90 * dis_for_right_angle
        rotatory_base.targetPosition =
            calculate_distance(distance, ROTATORY_BASE, REV_TICKS_PER_REV)
        rotatory_base.power = power
    }

    //function for the rotational movement, takes degrees and converts into distance
    fun rotate(degrees: Double, power: Double, dir: String) {

        //constant used for distance, for every 90 degree angle turn, the robot needs 55 cm
        val dis_for_right_angle = 54.5
        var distance = degrees / 90 * dis_for_right_angle
        run_to_position()
        if (dir == "right") {
            distance = distance
        } else if (dir == "left") {
            distance = -distance
        }
        front_left.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        front_right.targetPosition =
            calculate_distance(-distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_left.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_right.targetPosition =
            calculate_distance(-distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        set_power(power)
    }

    fun forward(distance: Double, power: Double) {
        var distance = distance
        distance = distance * 0.934579439252
        front_left.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_left.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        front_right.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        back_right.targetPosition =
            calculate_distance(distance, WHEEL_CIRCUMFERENCE, GO_TICKS_PER_REV)
        run_to_position()
        set_power(power)
    }

    fun grab_cone() {
        c1.position = 0.4
        c2.position = 0.0
    }

    fun release_cone() {
        c1.position = 0.0
        c2.position = 0.4
    }

    fun sliderup(distance: Double, power: Double) {
        //distance = distance;
        slider.targetPosition = calculate_distance(distance, SLIDER_WHEEL, GO_TICKS_PER_REV)
        slider.mode = DcMotor.RunMode.RUN_TO_POSITION
        slider.power = power
    }

    fun sliderdown(distance: Double, power: Double) {
        var distance = distance
        distance = -distance
        slider.targetPosition = calculate_distance(distance, SLIDER_WHEEL, GO_TICKS_PER_REV)
        slider.mode = DcMotor.RunMode.RUN_TO_POSITION
        slider.power = power
    }

    fun armextend(distance: Double, power: Double) {
        arm.targetPosition = calculate_distance(distance, 0.0, GO_TICKS_PER_REV)
        arm.mode = DcMotor.RunMode.RUN_TO_POSITION
        arm.power = power
    }

    fun armretract(distance: Double, power: Double) {
        var distance = distance
        distance = -distance
        arm.targetPosition = calculate_distance(distance, 0.0, GO_TICKS_PER_REV)
        arm.mode = DcMotor.RunMode.RUN_TO_POSITION
        arm.power = power
    }

    fun grab_claw() {
        c1.position = 0.4
        c2.position = 0.0
    }

    fun release_claw() {
        c1.position = 0.0
        c2.position = 0.4
    }

    fun set_power(power: Double) {
        front_left.power = power
        front_right.power = power
        back_left.power = power
        back_right.power = power
    }

    fun stop_wheel_motors() {
        front_right.power = 0.0
        front_left.power = 0.0
        back_right.power = 0.0
        back_left.power = 0.0
    }

    fun stop_rotatory_base() {
        rotatory_base.power = 0.0
    }

    fun stop_slider() {
        slider.power = 0.0
    }

    fun stop() {
        stop_wheel_motors()
        stop_slider()
        rotatory_base.power = 0.0
        arm.power = 0.0
    }

    fun wait_wheel_motors() {
        //telemetry.addLine("waiting wheel motors");
        //telemetry.update();
        while (front_left.isBusy && front_right.isBusy && back_left.isBusy && back_right.isBusy) {
        }
    }

    fun wait_rotatory_base() {
        while (rotatory_base.isBusy) {
        }
    }

    fun wait_slider() {
        //telemetry.addLine("waiting slider motor");
        //telemetry.update();
        while (slider.isBusy) {
        }
    }

    fun wait_motors() {
        //telemetry.addLine("waiting ALL motors");
        //telemetry.update();
        while (front_left.isBusy && front_right.isBusy && back_left.isBusy && back_right.isBusy || rotatory_base.isBusy || slider.isBusy) {
        }
    }

    fun setRotBasePower(rotBasePower: Double) {
        rotBasePOwer = rotBasePower
    }

    companion object {
        private const val GO_TICKS_PER_REV = 537.7
        private const val PI = 3.14159265
        private const val WHEEL_CIRCUMFERENCE = PI * 3.7795
        private const val CONST_LATERAL_MOVEMENT = 1.136363636363
        private const val SLIDER_WHEEL = PI * 0.440944882
        private const val ROTATORY_BASE = PI * 5.90551181
        private const val REV_TICKS_PER_REV = 288.0
    }
}