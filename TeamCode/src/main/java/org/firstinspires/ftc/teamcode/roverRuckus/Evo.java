package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Evo", group = "real")
public class Evo extends Globulus{
    //
    Integer position;
    //
    public void runOpMode(){
        position = sater();
        //
        if (position == -1){
            moveToPosition(10, .3);
            //
            moveToPosition(-10, .3);
        }
        //
        sleep(500);
        //
        moveToPosition(-6, .3);
        //
        turnWithEncoder(-.3);//Don't change speed
        //
        while (getAngle() > 10 && opModeIsActive()){}
        //
        turnWithEncoder(0);
        //
        drive(.3);
        //
        while (!(jeep.getDistance(DistanceUnit.INCH) <  10)){}
        //
        drive(0);
        //
        turnWithGyro(25, -.4);
        //
        moveToPosition(20, .3);
        //
        turnWithGyro(15, -.3);
        //
        drive(.3);
        //
        while (!(jeep.getDistance(DistanceUnit.INCH) < 10)){}
        //
        drive(0);
        //
        releaseTheHounds.setPosition(1);
        //
        turnWithGyro(90, -.3);
        //
        moveToPosition(10, .3);
        //
        flapper.setPosition(1);
    }
}