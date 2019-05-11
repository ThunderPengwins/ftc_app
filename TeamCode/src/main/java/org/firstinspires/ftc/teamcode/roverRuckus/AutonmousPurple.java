package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "The Angry One", group = "Auto")
@Disabled
public class AutonmousPurple extends OmniAutoMode{
    @Override
    public void runOpMode() {
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        wall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wall");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        initGyro();
        configureMotors();
        toPosition();
        //
        waitForStartify();
////Unlatch
//        waitForStartify();
//        moveToPosition(5, 0.50);
//        sleep(1000);
////Go Forward, Turn
//        turnWithGyro(180, 0.50);
//        moveToPosition(5, 0.50);
//        turnWithGyro(45, 0.50);
//
////Start Vuforia Scanning
//        //todo ERIC PUT VURFORIA HERE
//
////When Found Knock The One Off
//
////Drive To Depo
//        moveToPosition(5, 0.5);
//Go To The Crater
        //turnWithGyro(45,0.10);
        configureMotors();
        //
        withoutEncoder();
//        telInit("complete");
        //
        waitForStart();
        //
        drive(.2);
        //
        while (jeep.getDistance(DistanceUnit.INCH)> 5){
            if (wall.getDistance(DistanceUnit.INCH) < 5){
                telMove("Too close!");
                right.setPower(right.getPower() - .01);
            } else if (wall.getDistance(DistanceUnit.INCH) > 8 || wall.getDistance(DistanceUnit.INCH) == DistanceUnit.infinity){
                telMove("Too far!");
                left.setPower(left.getPower() - .01);
            } else {
                telMove("Just Right");
                drive(.2);
            }
            telemetry.update();
        }
        countdown(10);

        if (jeep.getDistance(DistanceUnit.INCH)> 5){
            while (jeep.getDistance(DistanceUnit.INCH) > 5){
                if (wall.getDistance(DistanceUnit.INCH) < 5){
                    telMove("Too close!");
                    right.setPower(right.getPower() - .01);
                } else if (wall.getDistance(DistanceUnit.INCH) > 8 || wall.getDistance(DistanceUnit.INCH) == DistanceUnit.infinity){
                    telMove("Too far!");
                    left.setPower(left.getPower() - .01);
                } else {
                    telMove("Just Right");
                    drive(.2);
                }
                telemetry.update();
            }
        }

    }
}
//This is not accurate it needs to be tested