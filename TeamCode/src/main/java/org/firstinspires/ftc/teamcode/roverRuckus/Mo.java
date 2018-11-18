package org.firstinspires.ftc.teamcode.roverRuckus;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "Mo", group = "real")
public class Mo extends OmniMode {
    //
    ModernRoboticsI2cRangeSensor jeep;
    DcMotor vertical;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    Servo flapper;
    Servo releaseTheHounds;
    //
    //closed = 1
    //open = 0.3
    //
    static final Double closed = 0.5;
    static final Double open = 0.1;
    //
    public void runOpMode(){
        //
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
        Double power = .5;
        Float leftC;
        Float rightC = 0F;
        Boolean auto = false;
        Integer direction = 1;
        Boolean powerP = false;
        Double position = 0.0;
        Integer movement = 0;
        //
        configureMotors();
        telInit("complete");
        //
        waitForStartify();
        //
        Double rightPower = .5;
        Double leftPower = .5;
        //
        while (opModeIsActive()){
            //
            left.setPower(-gamepad1.left_stick_y * leftPower);
            right.setPower(-gamepad1.right_stick_y * rightPower);
            //
            if (gamepad1.right_trigger > 0){
                rightPower = .2;
                telemetry.addData("Trigger", "Right");
            } else if (gamepad1.right_bumper){
                rightPower = .05;
                telemetry.addData("Bumper", "Right");
            } else{
                rightPower = .5;
                telemetry.addData("No action", "Right");
            }
            //
            if (gamepad1.left_trigger > 0){
                leftPower = .2;
                telemetry.addData("Trigger", "Left");
            }else if (gamepad1.left_bumper){
                leftPower = .05;
                telemetry.addData("Bumper", "Left");
            }else {
                leftPower = .5;
                telemetry.addData("No action", "Left");
            }
            //
            telemetry.addData("distance", jeep.getDistance(DistanceUnit.INCH));
            telemetry.update();
            //
            leftC = -gamepad2.left_stick_y;//left y
            rightC = -gamepad2.right_stick_x;//right x
            //
            //<editor-fold desc="Set auto move">
            if (gamepad2.y) {
                direction = 1;
                if (!auto) {
                    auto = true;
                }
            } else if (gamepad2.x) {
                direction = -1;
                if (!auto) {
                    auto = true;
                }
            }
            //</editor-fold>
            //<editor-fold desc="Change power">
            //</editor-fold>
            //
            //<editor-fold desc="set moter power">
            if (((!up.getState() && leftC > 0) || (!down.getState() && leftC < 0)) && !auto) {
                vertical.setPower(0);
            } else if (auto) {
                if (!((up.getState() && direction == 1) || (down.getState() && direction == -1))) {
                    vertical.setPower(power * direction);
                } else {
                    vertical.setPower(0);
                    auto = false;
                }
            }
            //
            if (gamepad2.a) {
                position = open;
            } else if (gamepad2.b){
                position = closed;
            }
            //
            if (gamepad2.dpad_right){
                releaseTheHounds.setPosition(.4);
            } else if(gamepad2.dpad_left){
                releaseTheHounds.setPosition(0);
            }
            //
            if (gamepad2.dpad_up){
                flapper.setPosition(1);
            } else if (gamepad2.dpad_down){
                flapper.setPosition(0);
            }
            //
            //
            latch.setPosition(position);
            //</editor-fold>
            //
            telemetry.addData("power", leftC);
            telemetry.addData("game", -gamepad2.left_stick_y);
            telemetry.addData("position", position);
            telemetry.addData("up", up.getState());
            telemetry.addData("down", down.getState());
            telemetry.addData("auto?", auto);
            telemetry.update();

        }
    }
}
