package org.firstinspires.ftc.teamcode.relicrecoveryv2;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.HardwareSensorMap;
import org.firstinspires.ftc.teamcode.TouchSensorCheck;

/**
 * Created by Nora and Eric on 12/30/2017.
 */

public class PengwinWing {
    //motors
    DcMotor motorOne;
    DcMotor motorTwo;
    //servos
    Servo servoOne;
    Servo servoTwo;
    Servo servoThree;
    Servo servoFour;
    //sensors
    ColorSensor upperColor;
    ColorSensor lowerColor;
    TouchSensor slHome;
    TouchSensor slUp;
    TouchSensor extenderOut;
    TouchSensor extenderIn;
    TouchSensor upperGrabberFilled;
    TouchSensor lowerGrabberFilled;
    //setting power and whatnot
    //motors
    double motorOnePower;
    double motorTwoPower;
    //servos
    double servoOnePower;
    double servoTwoPower;
    //sensors
    boolean upperColorReturn;
    boolean lowerColorReturn;
    boolean slHomeReturn;
    boolean slUpReturn;
    boolean extenderOutReturn;
    boolean extenderInReturn;
    boolean upperGrabberFilledReturn;
    boolean lowerGrabberFilledReturn;

    public PengwinWing(HardwareMap hardwareMap){
        //sets the things
        motorOne = hardwareMap.dcMotor.get("motor1");
        motorTwo = hardwareMap.dcMotor.get("motor2");
        servoOne = hardwareMap.servo.get("servo1");
        servoTwo = hardwareMap.servo.get("servo2");
        servoThree = hardwareMap.servo.get("servo3");
        servoFour = hardwareMap.servo.get("servo4");
        upperColor = hardwareMap.colorSensor.get("color1");
        lowerColor = hardwareMap.colorSensor.get("color2");
        slHome = hardwareMap.touchSensor.get("touch1");
        slUp = hardwareMap.touchSensor.get("touch2");
        extenderOut = hardwareMap.touchSensor.get("touch3");
        extenderIn = hardwareMap.touchSensor.get("touch4");
        upperGrabberFilled = hardwareMap.touchSensor.get("touch5");
        lowerGrabberFilled = hardwareMap.touchSensor.get("touch6");
}

}