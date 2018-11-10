package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Wall-E", group = "Auto")
public class Wall_E extends OmniAutoMode{
    //
    public void runOpMode(){
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //
        configureMotors();
        //
        toPosition();
        //
        telInit("gyro");
        initGyro();
        //
        telInit("complete");
        //
        waitForStartify();
        //
        telMove("in progress");
        turnWithGyro(180, -.1);
        sleep(1000);
        turnWithGyro(270, .1);
        //
        telMove("complete");
    }
    //
}
