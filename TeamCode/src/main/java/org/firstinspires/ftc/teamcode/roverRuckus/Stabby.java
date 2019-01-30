package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stabby", group = "TeleOp")
@Disabled
public class Stabby extends OmniMode {

    //jk don't use this program

    DcMotor sirStabby;
    Servo sabbiesClaw;
    DigitalChannel stabbyUp;
    DigitalChannel stabbyDown;
    Servo latch;
//    DcMotor verticala;


    public void runOpMode(){

        telInit("hardware");
        sirStabby = hardwareMap.dcMotor.get("verticala");
        sabbiesClaw = hardwareMap.servo.get("latch");
        stabbyDown = hardwareMap.get(DigitalChannel.class, "down");
        stabbyUp = hardwareMap.get(DigitalChannel.class, "up");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        Float stabbyVertical = 0F;
        Float sabbiesGrabber = 0F;
        Double power = .5;
        Boolean auto = false;
        Integer direction = 1;
        Boolean powerP = false;
        Double position = 0.0;
        Boolean powerO = false;
        configureMotors();
        sirStabby.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sirStabby.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStartify();

        while (opModeIsActive()){
//        stabbyVertical = -gamepad2.left_stick_y;
//        sabbiesGrabber = -gamepad2.right_stick_x;


            if (gamepad2.b && !powerP) {
                power += .1;
                powerP = true;
            } else if (gamepad2.a && !powerP) {
                power -= .1;
                powerP = true;
            } else if (!(gamepad2.a && gamepad2.b) && powerP) {
                powerP = false;
            }

            if (((!stabbyUp.getState() == false) || (!stabbyDown.getState() == false)) && !auto) {
                sirStabby.setPower(0);
            }

            telemetry.addData("Switch", stabbyDown.getState());
            telemetry.update();

            if (gamepad2.x && !powerO) {
                powerO = true;
                position += .1;
            } else if (gamepad2.y && !powerO) {
                powerO = true;
                position -= .1;
            } else if (!(gamepad2.y && gamepad2.b) && powerO) {
                powerO = false;
            }

            sabbiesClaw.setPosition(position);

}}}
