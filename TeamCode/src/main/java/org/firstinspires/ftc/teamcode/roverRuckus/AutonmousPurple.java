package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "The Angry One", group = "Auto")

public class AutonmousPurple extends OmniAutoMode{
    @Override
    public void runOpMode() {
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        sideSensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        frontRangesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        initGyro();
        configureMotors();
        toPosition();


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
        turnWithGyro(45,0.50);
        telemetry.addData("Side Range", "%.2f cm", sideSensorDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Range", "%.2f cm", frontRangesensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        while (frontRangesensor.getDistance(DistanceUnit.CM)<10){
        if (sideSensorDistance.getDistance(DistanceUnit.CM)<5){
            turnWithGyro(10, 0.5);
        }
        else{
            moveToPosition(5, 0.5);
        }}

//        turnWithGyro(45,0.50);
//        moveToPosition(5, 0.50);
    }
}
//This is not accurate it needs to be tested