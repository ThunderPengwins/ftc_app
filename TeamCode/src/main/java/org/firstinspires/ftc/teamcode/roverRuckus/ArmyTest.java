package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmyTest", group = "test")
public class ArmyTest extends OmniMode {
    //
    static final double pr1pow = .2;
    static final double liepow = .2;
    //
    static final double pr1vangle = 1.0;
    static final double lievangle = 1.0;
    //
    static final List<Double> voltage = new ArrayList<Double>(){{
        add(.002);
        add(.139);
        add(.279);
        add(.401);
        add(.515);
        add(.643);
        add(.743);
        add(.875);
        add(1.006);
        add(1.125);
        add(1.302);
        add(1.512);
        add(1.616);
        add(1.87);
        add(2.118);
        add(2.423);
        add(2.794);
        add(3.311);
        add(3.334);
    }};
    //
    DcMotor pr1vate;
    DcMotor lieutenant;
    //
    AnalogInput cinput;
    AnalogInput dinput;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="HardwareMap">
        pr1vate = hardwareMap.dcMotor.get("pr1vate");//change in phones
        lieutenant = hardwareMap.dcMotor.get("lieutenant");
        cinput = hardwareMap.analogInput.get("cinput");
        dinput = hardwareMap.analogInput.get("dinput");
        //</editor-fold>
        //
        waitForStartify();
        //
        while (opModeIsActive()) {
            //
            //<editor-fold desc="Set auto move">
            telemetry.addData("cinput", cinput.getVoltage() + ", degrees: " + voltogres(cinput.getVoltage()));
            telemetry.addData("dinput", dinput.getVoltage() + ", degrees: " + voltogres(dinput.getVoltage()));
            telemetry.update();
        }
    }
    //
    public double voltogres(double volts){
        int n;
        for (n = 0; n < voltage.size() && voltage.get(n) < volts; n++){}
        //
        return (15 / (voltage.get(n) - voltage.get(n - 1))) * (volts - voltage.get(n - 1));
    }
}
