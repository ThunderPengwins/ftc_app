package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * @author Eric
 */

@TeleOp (name = "Moby", group = "real")
public class Moby extends OmniMode {
    //<editor-fold desc="hardware">
    /**
     * <h1>Hardware</h>
     * <p>Here is where we create any piece of hardware that
     * wasn't already created in the class above, OmniMode.
     * Later on, we do the hardware map, but that can't be done
     * until we say:</p>
     * @code runOpMode(){}
     */
    ModernRoboticsI2cRangeSensor jeep;
    DcMotor verticala;
    DcMotor verticalb;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    //</editor-fold>
    //
    static final Double open = 0.7;
    static final Double closed = 0.45;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="Init">
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        verticala = hardwareMap.dcMotor.get("verticala");
        verticalb = hardwareMap.dcMotor.get("verticalb");
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        //
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        verticala.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticala.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticala.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalb.setDirection(DcMotorSimple.Direction.REVERSE);
        //</editor-fold>
        //
        waitForStartify();
        //
        //<editor-fold desc="Variables">
        Integer direction = 0;//lifter direction
        Double rightPower = .5;//right motor power
        Double leftPower = .5;//left motor power
        Integer front = 1;//front side of robot
        Integer ready = 1;
        //
        configureMotors();
        telInit("complete");
        //</editor-fold>
        //
        waitForStartify();
        //
        while (opModeIsActive()) {
            //
            left.setPower(-gamepad1.left_stick_y * leftPower * front);
            right.setPower(-gamepad1.right_stick_y * rightPower * front);
            //
            //<editor-fold desc="Power Levels">
            if (gamepad1.right_trigger > 0) {
                rightPower = .2;
                telemetry.addData("Trigger", "Right");
            } else if (gamepad1.right_bumper) {
                rightPower = .05;
                telemetry.addData("Bumper", "Right");
            } else {
                rightPower = .5;
                telemetry.addData("No action", "Right");
            }
            //
            if (gamepad1.left_trigger > 0) {
                leftPower = .2;
                telemetry.addData("Trigger", "Left");
            } else if (gamepad1.left_bumper) {
                leftPower = .05;
                telemetry.addData("Bumper", "Left");
            } else {
                leftPower = .5;
                telemetry.addData("No action", "Left");
            }
            //</editor-fold>
            //
            //<editor-fold desc="Lifter">
            if (!((up.getState() && direction == 1) || (down.getState() && direction == -1)) && !(gamepad2.right_trigger > 0) && !(direction == 0)) {
                verticala.setPower(0.5 * direction);
                verticalb.setPower(0.5 * direction);
            } else {
                verticala.setPower(0);
                verticalb.setPower(0);
                direction = 0;
            }
            //
            if (gamepad2.y){
                direction = 1;
            } else if (gamepad2.x){
                direction = -1;
            }
            //</editor-fold>
            //
            //<editor-fold desc="Latch">
            if (gamepad2.a) {
                latch.setPosition(closed);
            } else if (gamepad2.b) {
                latch.setPosition(open);
            }
            //</editor-fold>
            //
            //<editor-fold desc="Front">
            if (gamepad1.b) {
                if (front == 1 && ready == 1) {
                    front = -1;
                } else if (front == -1 && ready == 1) {
                    front = 1;
                }
                ready = 0;
            } else {
                ready = 1;
            }
            //</editor-fold>
            //
            //<editor-fold desc="Telemetry">
            telemetry.addData("ready?", ready);
            telemetry.addData("front", front);
            telemetry.addData("distance", jeep.getDistance(DistanceUnit.INCH));
            telemetry.addData("up", up.getState());
            telemetry.addData("down", down.getState());
            telemetry.update();
            //</editor-fold>
        }
    }
}