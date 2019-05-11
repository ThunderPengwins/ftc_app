package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Spicy Mice", group = "test")
public class SpicyMice extends OmniAutoMode {
    //
    DcMotor pr1vate;
    DcMotor lieutenant;
    //
    public void runOpMode() {
        //
        telInit("hardware");
        pr1vate = hardwareMap.dcMotor.get("pr1vate");
        lieutenant = hardwareMap.dcMotor.get("lieutenant");
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pr1vate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lieutenant.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pr1vate.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        telInit("complete");
        waitForStartify();
        //
        lieutenant.setPower(.3);
        //
        for (int n = 0; n < 22; n++) {
            pr1vate.setPower(n * .05);
            telemetry.addData("Power", (n * .05));
            telemetry.update();
            sleep(2000);
        }
    }
}
