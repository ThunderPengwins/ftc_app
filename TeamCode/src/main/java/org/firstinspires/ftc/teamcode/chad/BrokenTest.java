package org.firstinspires.ftc.teamcode.chad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Created by Eric on 6/10/2018.

@TeleOp(name="brokenTest",group="chad") //Step 1, @Teleop, name & group, Okay to import

public class BrokenTest extends LinearOpMode{ //Step 2, extends..., light bulb>make abstract
    //
    DcMotor left;
    DcMotor right;
    //
    public void runOpMode() throws InterruptedException { //Step 4, runOpMode
        //
        boolean fullpower = false;
        //
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        waitForStart();
        //
        while (opModeIsActive()){
            //
            if (gamepad1.dpad_up){
                fullpower = true;
            }else if (gamepad1.dpad_down){
                fullpower = false;
            }
            if (fullpower) {
                right.setPower(-gamepad1.right_stick_y );
                left.setPower(-gamepad1.left_stick_y);
            } else{
                right.setPower(-gamepad1.right_stick_y * .5);
                left.setPower(-gamepad1.left_stick_y * .5);
            }
            telemetry.addData("left", gamepad1.right_stick_y);
            telemetry.addData("right", gamepad1.left_stick_y);
            telemetry.update();
            //
        }
        //
    }
}
