package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Spicy Dice", group = "test")
public class SpicyDice extends OmniAutoMode {
    //
    ModernRoboticsI2cRangeSensor jep;
    ModernRoboticsI2cRangeSensor ajay;
    //
    public void runOpMode() {
        //
        telInit("hardware");
        //
        jep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jep");
        ajay = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ajay");
        //
        telInit("complete");
        waitForStartify();
        //
        while (opModeIsActive()){
            telemetry.addData("jep", jep.getDistance(DistanceUnit.INCH));
            telemetry.addData("ajay", ajay.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
