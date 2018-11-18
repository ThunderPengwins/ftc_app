package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Spicy Rice", group = "test")
public class SpicyRice extends OmniAutoMode {
    //
    public void runOpMode() {
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        configureMotors();
        //
        telInit("Gyro");
        initGyro();
        //
        withEncoder();
        //
        telInit("complete");
        waitForStartify();
        //
        moveToPosition(10, .2);
    }
}
