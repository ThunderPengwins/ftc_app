package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FolloWall", group = "test")
public class FolloWall extends OmniAutoMode{
    //
    ModernRoboticsI2cRangeSensor jeep;
    DistanceSensor wall;
    //
    public void runOpMode(){
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        wall = hardwareMap.get(DistanceSensor.class, "wall");
        //
        configureMotors();
        //
        initGyro();
        //
        withoutEncoder();
        telInit("complete");
        //
        waitForStart();
        //
        drive(.2);
        //
        while (jeep.getDistance(DistanceUnit.INCH) > 5){
            if (getAngle() < 40){
                telMove(Double.toString(getAngle()));
                right.setPower(right.getPower() - .01);
            } else if (getAngle() > 50){
                telMove(Double.toString(getAngle()));
                left.setPower(left.getPower() - .01);
            } else {
                telMove("Just Right");
                drive(.2);
            }
        }
        //
        drive(0);
    }
}
