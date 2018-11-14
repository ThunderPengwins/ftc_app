package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stabby", group = "TeleOp")

public class Stabby extends OmniMode {

    DcMotor sirStabby;
    Servo sabbiesClaw;



    public void runOpMode(){

        telInit("hardware");
        sirStabby = hardwareMap.dcMotor.get("Stabby");
        sabbiesClaw = hardwareMap.servo.get("Stop");
        configureMotors();
        telInit("complete");
        waitForStartify();



}}
