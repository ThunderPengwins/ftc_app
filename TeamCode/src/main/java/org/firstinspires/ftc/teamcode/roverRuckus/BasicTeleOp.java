package org.firstinspires.ftc.teamcode.roverRuckus;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "BasicTeleOp", group = "usable")
public class BasicTeleOp extends OmniMode {
    //
    ModernRoboticsI2cRangeSensor jeep;
    //
    public void runOpMode(){
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
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
            if (jeep.getDistance(DistanceUnit.INCH) > 10 || ((-gamepad1.left_stick_y < 0) && (-gamepad1.right_stick_y < 0))) {
                left.setPower(-gamepad1.left_stick_y * leftPower);
                right.setPower(-gamepad1.right_stick_y * rightPower);
            } else {
                left.setPower(0);
                right.setPower(0);
            }
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
            telemetry.addData("distance", jeep.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
