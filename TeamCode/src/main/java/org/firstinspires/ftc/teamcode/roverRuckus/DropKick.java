package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Drop Kick", group = "test")
public class DropKick extends Globulus{
    ModernRoboticsI2cRangeSensor jeep;
    DcMotor verticala;
    DcMotor verticalb;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    Servo flapper;
    Servo releaseTheHounds;
    //</editor-fold>
    //
    static final Double open = .7;
    static final Double closed = 0.45;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="Init">
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        verticala = hardwareMap.dcMotor.get("verticala");//change in phones
        verticalb = hardwareMap.dcMotor.get("verticalb");//change in phones
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        releaseTheHounds = hardwareMap.servo.get("release");
        flapper = hardwareMap.servo.get("flapper");
        //
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        verticala.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticala.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticala.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalb.setDirection(DcMotorSimple.Direction.REVERSE);
        //</editor-fold>
        //
        verticala.setPower(1.0);
        verticalb.setPower(1.0);
        //
        while (!up.getState()) {
            telemetry.addData("up", up.getState());
            telemetry.update();
        }
        //
        verticala.setPower(0);
        verticalb.setPower(0);
        //
        latch.setPosition(open);
        //
        sleep(1000);
    }
}