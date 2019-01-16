package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmyTest", group = "test")
public class ArmyTest extends OmniMode {
    //
    DcMotor soldier;
    DcMotor lieutenant;
    //
    //closed = 1
    //open = 0.3
    //
    static final Double closed = .5;
    static final Double open = 0.1;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="HardwareMap">
        soldier = hardwareMap.dcMotor.get("soldier");//change in phones
        lieutenant = hardwareMap.dcMotor.get("lieutenant");
        //
        //</editor-fold>
        //
        waitForStartify();
        //
        //<editor-fold desc="Variables">
        Double power = 1.0;
        Float leftC;
        Float rightC = 0F;
        Boolean auto = false;
        Integer direction = 1;
        Boolean powerP = false;
        Double position = 0.5;
        Integer movement = 0;
        //</editor-fold>
        //
        while (opModeIsActive()) {
            //
            leftC = -gamepad2.left_stick_y;//left y
            rightC = -gamepad2.right_stick_x;//right x
            //
            //<editor-fold desc="Set auto move">
            if (gamepad2.y) {
                direction = 1;
                if (!auto) {
                    auto = true;
                }
            } else if (gamepad2.x) {
                direction = -1;
                if (!auto) {
                    auto = true;
                }
            }
            //</editor-fold>
            //
            //<editor-fold desc="set moter power">

            //
            if (gamepad2.a) {
                position = open;
            } else if (gamepad2.b){
                position = closed;
            }
            //
            telemetry.update();
        }
    }
    //
}
