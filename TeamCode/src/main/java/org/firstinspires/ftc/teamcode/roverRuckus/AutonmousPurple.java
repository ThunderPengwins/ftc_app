package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "The Angry One", group = "Auto")

public class AutonmousPurple extends OmniAutoMode{
    @Override
    public void runOpMode() {
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        initGyro();
        configureMotors();
        toPosition();

//Unlatch
        waitForStartify();
        moveToPosition(5, 0.50);
        sleep(1000);
//Go Forward, Turn
        turnWithGyro(180, 0.50);
        moveToPosition(5, 0.50);
        turnWithGyro(45, 0.50);

//Start Vuforia Scanning
        //todo ERIC PUT VURFORIA HERE

//When Found Knock The One Off

//Drive To Depo
        moveToPosition(5, 0.5);
//Go To The Crater
        turnWithGyro(45,0.50);
        moveToPosition(5, 0.50);
    }
}
//This is not accurate it needs to be tested