package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "Turving", group = "test")
public class Turving extends OmniAutoMode {
    //
    public void runOpMode() {
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        configureMotors();
        //
        waitForStartify();
        //
        turveRightByPoint(0.0, 20.0, 10.0, .8);
    }
    //
    public void turveLeftByPoint(Double x, Double y, Double clearance, Double speed){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        sleep(1000);
        //
        Double leftMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double rightMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor * countify));
        int leftd = (int)(Math.round(leftMotor * countify));
        //
        telemetry.addData("left motor", leftMotor + ", " + leftd);
        telemetry.addData("right motor", rightMotor + ", " + rightd);
        telemetry.update();
        //[
        sleep(1000);
        //
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(right.getCurrentPosition() + rightd);
        left.setTargetPosition(left.getCurrentPosition() + leftd);
        //
        right.setPower(speed);
        left.setPower((leftMotor / rightMotor) * speed);
        //
        while (right.isBusy()){
            telemetry.addData("left motor", leftMotor);
            telemetry.addData("right motor", rightMotor);
            telemetry.addData("left power", ((leftMotor / rightMotor) * .4) + ", reality: " + left.getPower());
            telemetry.addData("right power", .4 + ", reality: " + right.getPower());
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
        //
    }
    //
    public void turveRightByPoint(Double x, Double y, Double clearance, Double speed){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        sleep(1000);
        //
        Double rightMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double leftMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor * countify));
        int leftd = (int)(Math.round(leftMotor * countify));
        //
        telemetry.addData("left motor", leftMotor + ", " + leftd);
        telemetry.addData("right motor", rightMotor + ", " + rightd);
        telemetry.update();
        //[
        sleep(1000);
        //
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(right.getCurrentPosition() + rightd);
        left.setTargetPosition(left.getCurrentPosition() + leftd);
        //
        left.setPower(speed);
        right.setPower((rightMotor / leftMotor) * speed);
        //
        while (right.isBusy()){
            telemetry.addData("left motor", leftMotor);
            telemetry.addData("right motor", rightMotor);
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
        //
    }
}
