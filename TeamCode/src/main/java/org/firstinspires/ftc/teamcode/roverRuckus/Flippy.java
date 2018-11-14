package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Double.NaN;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Flippy", group = "Auto")

public class Flippy extends OmniAutoMode{
    public void runOpMode() {
        telInit("hardware");
        initGyro();
        Servo flippy;
        flippy = hardwareMap.get(Servo.class, "flippy");
        int wow = 0;
        waitForStartify();

        while (opModeIsActive()){
            wow+=1;
            flippy.setPosition(wow);
            telemetry.addData("Degrees", flippy.getPosition());
            telemetry.update();
            waitify(1000);
        }
}}
//Nora is cool