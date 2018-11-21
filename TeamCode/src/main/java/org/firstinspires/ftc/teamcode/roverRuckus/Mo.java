package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Mo", group = "real")
public class Mo extends OmniMode {
    //<editor-fold desc="hardware">
    ModernRoboticsI2cRangeSensor jeep;
    DcMotor vertical;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    Servo flapper;
    Servo releaseTheHounds;
    //</editor-fold>
    //
    static final Double closed = 0.5;
    static final Double open = 0.1;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="Init">
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        vertical = hardwareMap.dcMotor.get("vertical");//change in phones
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        releaseTheHounds = hardwareMap.servo.get("release");
        flapper = hardwareMap.servo.get("flapper");
        //
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //</editor-fold>
        //
        waitForStartify();
        //
        //<editor-fold desc="Variables">
        Integer direction = 1;//lifter direction
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
                vertical.setPower(0.5 * direction);
            } else {
                vertical.setPower(0);
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
                latch.setPosition(open);
            } else if (gamepad2.b) {
                latch.setPosition(closed);
            }
            //</editor-fold>
            //
            //<editor-fold desc="Surrender">
            if (gamepad2.dpad_up) {
                flapper.setPosition(1);
            } else if (gamepad2.dpad_down) {
                flapper.setPosition(0);
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
            telemetry.addData("direction", -gamepad2.left_stick_y);
            telemetry.addData("up", up.getState());
            telemetry.addData("down", down.getState());
            telemetry.update();
            //</editor-fold>
        }
    }
}