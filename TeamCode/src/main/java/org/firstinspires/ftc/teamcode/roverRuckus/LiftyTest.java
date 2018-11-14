package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LiftyTest", group = "test")
public class LiftyTest extends OmniMode {
    //
    DcMotor vertical;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;

    //
    public void runOpMode() {
        //
        //<editor-fold desc="HardwareMap">
        vertical = hardwareMap.dcMotor.get("vertical");//change in phones
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        //
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //</editor-fold>
        //
        waitForStartify();
        //
        //<editor-fold desc="Variables">
        Double power = .5;
        Float leftC = 0F;
        Float rightC = 0F;
        Boolean auto = false;
        Integer direction = 1;
        Boolean powerP = false;
        Double position = 0.0;
        Boolean powerO = false;
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
            //<editor-fold desc="Change power">
            if (gamepad2.b && !powerP) {
                power += .1;
                powerP = true;
            } else if (gamepad2.a && !powerP) {
                power -= .1;
                powerP = true;
            } else if (!(gamepad2.a && gamepad2.b) && powerP) {
                powerP = false;
            }
            //</editor-fold>
            //
            //<editor-fold desc="set moter power">
            if (((!up.getState() && leftC > 0) || (!down.getState() && leftC < 0)) && !auto) {
                vertical.setPower(leftC);
            } else if (auto) {
                if (!((up.getState() && gamepad2.y) || (down.getState() && gamepad2.x))) {
                    vertical.setPower(power * direction);
                } else {
                    vertical.setPower(0);
                    auto = false;
                }
            }

            if (gamepad2.x && !powerO) {
                powerO = true;
                position += .1;
            } else if (gamepad2.y && !powerO) {
                powerO = true;
                position -= .1;
            } else if (!(gamepad2.y && gamepad2.b) && powerO) {
                powerO = false;
            }
            //
            latch.setPosition(position);
            //</editor-fold>
            //
            telemetry.addData("power", leftC);
            telemetry.addData("game", -gamepad2.left_stick_y);
            telemetry.addData("position", position);
            telemetry.addData("up", up.getState());
            telemetry.addData("down", down.getState());
            telemetry.addData("go?", powerO);
            telemetry.addData("auto?", auto);
            telemetry.update();
        }
    }
    //
}
