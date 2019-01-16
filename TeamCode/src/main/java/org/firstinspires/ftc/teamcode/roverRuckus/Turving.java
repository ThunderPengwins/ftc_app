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
        turveLeftByPoint(0.0, 10.0, 0.1);
    }
    //
    public void turveLeftByPoint(Double x, Double y, Double clearance){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        sleep(5000);
        //
        Double rightMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double leftMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor*countify));
        int leftd = (int)(Math.round(leftMotor*countify));
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(rightd);
        left.setTargetPosition(leftd);
        //
        right.setPower(.2);
        left.setPower((leftMotor / rightMotor) * .2);
        //
        telemetry.addData("left motor", leftMotor);
        telemetry.addData("right motor", rightMotor);
        telemetry.addData("left power", ((leftMotor / rightMotor) * .2) + ", reality: " + left.getPower());
        telemetry.addData("right power", .2 + ", reality: " + right.getPower());
        telemetry.update();
        //
        while (right.isBusy()){}
    }
}
