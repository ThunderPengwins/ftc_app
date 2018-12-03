package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Eva", group = "real")
public class Eva extends Globulus {
    //
    Integer position;
    //
    public void runOpMode() {
        position = sarker();
        //
        turnWithGyro(38, -.3);
        //
        if (position == 0){
            moveToPosition(-21, .3);
            //
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(35, .3);
        }else if (position == -1) {
            moveToPosition(-11, .3);
            //
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(25, .3);
        }else{
            moveToPosition(-19, .3);
            //
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(50, .3);
        }
        //
        followGyro(48);
        //
        turnWithGyro(180, -.3);
        //
        drive(.3);
        flapper.setPosition(1);
        //
        sleep(2000);
        drive(0);
    }
}