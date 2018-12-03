package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Flippy", group = "Auto")
@Disabled
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
            countdown(1);
        }
}}
//Nora is cool