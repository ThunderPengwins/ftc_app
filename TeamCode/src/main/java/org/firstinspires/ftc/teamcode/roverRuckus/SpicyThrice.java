package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Spicy Thrice", group = "test")
public class SpicyThrice extends OmniAutoMode {
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
        for (int n = 0; n < 30; n++) {
            turn(.3 - (n * .01));
            telemetry.addData("Power", .3 - (n * .01));
            telemetry.update();
            sleep(1000);
        }
    }
}
