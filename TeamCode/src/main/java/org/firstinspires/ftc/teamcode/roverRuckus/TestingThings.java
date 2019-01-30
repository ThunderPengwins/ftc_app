package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "obsticalcourse", group = "Tele-Op")

public class TestingThings extends OmniAutoMode {
    @Override
    public void runOpMode() {
        wallFollower(4,10);
        telemetry.update();
    }
}
