package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Wall-G", group = "real")
public class Wall_G extends Globulus{
    //
    Integer position;
    //
    public void runOpMode(){
        position = sater();
        //
        if (position == -1){
            moveToPosition(6, .3);
            //
            turnWithGyro(20, .3);
            //
            moveToPosition(5, .3);
        } else if (position == 1){
            moveToPosition(10, .3);
        }
        //
        flapper.setPosition(1);
        sleep(2000);
    }
}