package org.firstinspires.ftc.teamcode.roverRuckus;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "Wall-E", group = "real")
public class Wall_E extends Globulus{
    Integer position;
    public void runOpMode(){
        position = sarker();
        //
        if (position == 0) {
            moveToPosition(10, .3);
            //
            turnWithGyro(70, -.3);
        }else if (position == -1){
            turnWithGyro(80, -.3);
        }else{
            turnWithGyro(70, -.3);
        }
        //
        releaseTheHounds.setPosition(0);
        //
        moveToPosition(20, .3);
    }
}