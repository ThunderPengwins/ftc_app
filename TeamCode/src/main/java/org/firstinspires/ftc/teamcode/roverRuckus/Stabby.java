package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stabby", group = "TeleOp")

public class Stabby extends OmniMode {

    DcMotor sirStabby;
    Servo sabbiesClaw;
    DigitalChannel stabbyUp;
    DigitalChannel stabbyDown;
    Servo latch;
    DcMotor vertical;


    public void runOpMode(){

        telInit("hardware");
        sirStabby = hardwareMap.dcMotor.get("Stabby");
        sabbiesClaw = hardwareMap.servo.get("Stop");
        stabbyDown = hardwareMap.get(DigitalChannel.class, "down");
        stabbyUp = hardwareMap.get(DigitalChannel.class, "up");
        Float stabbyVertical = 0F;
        Float clawOpen = 0F;
        Double power = .5;
        Boolean auto = false;
        Integer direction = 1;
        Boolean powerP = false;
        Double position = 0.0;
        Boolean powerO = false;
        configureMotors();
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStartify();

        while (opModeIsActive()){
        stabbyVertical = -gamepad2.left_stick_y;
        clawOpen = -gamepad2.right_stick_x;


            if (gamepad2.y) {
                direction = 1;
                if (!auto) {
                    auto = true;
                }
            } else if (gamepad2.x) {
                direction = -1;
                if (!auto) {
                    auto = true;
                }
            }

            if (gamepad2.b && !powerP) {
                power += .1;
                powerP = true;
            } else if (gamepad2.a && !powerP) {
                power -= .1;
                powerP = true;
            } else if (!(gamepad2.a && gamepad2.b) && powerP) {
                powerP = false;
            }

            if (((!stabbyUp.getState() && stabbyVertical > 0) || (!stabbyDown.getState() && stabbyVertical < 0)) && !auto) {
                vertical.setPower(stabbyVertical);
            } else if (auto) {
                if (!((stabbyUp.getState() && gamepad2.y) || (stabbyDown.getState() && gamepad2.x))) {
                    vertical.setPower(power * direction);
                } else {
                    vertical.setPower(0);
                    auto = false;
                }
            }

            if (gamepad2.x && !powerO) {
                powerO = true;
                position += .1;
            } else if (gamepad2.y && !powerO) {
                powerO = true;
                position -= .1;
            } else if (!(gamepad2.y && gamepad2.b) && powerO) {
                powerO = false;
            }

            latch.setPosition(position);

}}}
