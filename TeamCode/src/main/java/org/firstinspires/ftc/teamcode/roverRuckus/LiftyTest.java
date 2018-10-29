package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "LiftyTest", group = "test")
public class LiftyTest extends OmniMode{
    //
    DcMotor vertical;
    TouchSensor down;
    TouchSensor up;
    Servo latch;
    //
    public void runOpMode(){
        //
        //<editor-fold desc="HardwareMap">
        vertical = hardwareMap.dcMotor.get("verticle");
        down = hardwareMap.touchSensor.get("down");
        up = hardwareMap.touchSensor.get("up");
        latch = hardwareMap.servo.get("latch");
        //
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        //</editor-fold>
        //
        while (opModeIsActive()){
            //
            leftC = -gamepad2.left_stick_y;//left y
            rightC = -gamepad2.right_stick_x;//right x
            //
            //<editor-fold desc="Set auto move">
            if (gamepad2.y){
                direction = 1;
                if (!auto){
                    auto = true;
                }
            } else if (gamepad2.x){
                direction = -1;
                if (!auto){
                    auto = true;
                }
            }
            //</editor-fold>
            //
            //<editor-fold desc="Change power">
            if (gamepad2.b && !powerP){
                power += .1;
                powerP = true;
            } else if (gamepad2.a && !powerP){
                power -= .1;
                powerP = true;
            } else if (!(gamepad2.a && gamepad2.b) && powerP){
                powerP = false;
            }
            //</editor-fold>
            //
            //<editor-fold desc="set moter power">
            if (((!up.isPressed() && leftC > 0) || (!down.isPressed() && leftC < 0)) && !auto){
                vertical.setPower(leftC);
            } else if (auto){
                if (!(up.isPressed() && down.isPressed())){
                    vertical.setPower(power * direction);
                } else{
                    vertical.setPower(0);
                    auto = false;
                }
            }
            //</editor-fold>
            //
            telemetry.addData("power", power);
        }
    }
    //
}
