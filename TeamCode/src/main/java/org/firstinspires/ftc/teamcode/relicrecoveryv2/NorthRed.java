package org.firstinspires.ftc.teamcode.relicrecoveryv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Eric on 1/6/2018.
 */

//@Autonomous(name="NorthRed", group="AutoMode")
public class NorthRed extends RelicAutoMode {
    @Override
    public void runOpMode() {
        //
        waitForStartify();
        //
        int someVariable = testOutMoving(10);
        //
        while (opModeIsActive()){
            telemetry.addData("result", someVariable);
            telemetry.update();
        }
    }
}
