package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp (name = "Drop Kick", group = "test")
public class DropKick extends Globulus{
    ModernRoboticsI2cRangeSensor jeep;
    //
    public void runOpMode() {
        //<editor-fold desc="Init">
        telInit("hardware");
        verticala = hardwareMap.dcMotor.get("verticala");//change in phones
        verticalb = hardwareMap.dcMotor.get("verticalb");//change in phones
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        //
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        verticala.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalb.setDirection(DcMotorSimple.Direction.REVERSE);
        //</editor-fold>
        //
        telInit("done");
        //
        waitForStartify();
        //
        verticala.setPower(-1.0);
        verticalb.setPower(-1.0);
        //
        while (!down.getState() && opModeIsActive()) {
            telemetry.addData("up", down.getState());
            telemetry.update();
        }
        //
        verticala.setPower(0);
        verticalb.setPower(0);
        //
        while (opModeIsActive()){}
    }
}