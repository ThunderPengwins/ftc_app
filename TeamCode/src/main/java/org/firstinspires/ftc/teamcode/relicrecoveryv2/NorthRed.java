package org.firstinspires.ftc.teamcode.relicrecoveryv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Eric on 1/6/2018.
 */
@Autonomous (name = "NarthRed", group = "Autos")
public class NorthRed extends RelicAutoMode {
    @Override
    public void runOpMode() {
        autoNorth(1);
    }
}