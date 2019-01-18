package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmyTest", group = "test")
public class ArmyTest extends OmniMode {
    //
    static final double pr1len = 14.5;
    static final double lielen = 20;
    //
    static final double pr1pow = .2;
    static final double liepow = .2;
    //
    static final double pr1vangle = 1.0;
    static final double lievangle = 1.0;
    //
    static final List<Double> voltage = new ArrayList<Double>(){{
        add(-.002);
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
        double c;
        double d;
        //
        List<Double> cords;
        List<Double> powers;
        //
        waitForStartify();
        //
        while (opModeIsActive()) {
            //
            c = voltogres(cinput.getVoltage());
            d = voltogres(dinput.getVoltage());
            //
            cords = xny(c, d);
            //
            powers = lumina(cords.get(0), cords.get(1), c, d);
            //
            //<editor-fold desc="Set auto move">
            telemetry.addData("cinput", round(cinput.getVoltage(), 2) + ", degrees: " + round(voltogres(cinput.getVoltage()), 2));
            telemetry.addData("dinput", round(dinput.getVoltage(), 2) + ", degrees: " + round(voltogres(dinput.getVoltage()), 2));
            telemetry.addData("x", round(cords.get(0), 2) + ", y: " + round(cords.get(1), 2));
            telemetry.addData("private's power", round(powers.get(0), 2));
            telemetry.addData("lieutenant's power", round(powers.get(1), 2));
            telemetry.update();
        }
    }
    //
    public double voltogres(double volts){
        int n;
        for (n = 0; n < voltage.size() && voltage.get(n) < volts; n++){}
        //
        return ((n * 15) / (voltage.get(n))) * (volts);
    }
    //
    public static List<Double> xny(double c, double da) {
        //B1*COS(RADIANS(G6))+B2*COS(RADIANS(H7-(90-G6)))
        //B1*SIN(RADIANS(G6))+B2*SIN(RADIANS(H7-(90-G6)))
        List<Double> xny = new ArrayList<>();
        //IF(B4-90>-1,B4-90,B4+270)
        double d;
        if (da - 90 > -1){
            d = da - 90;
        }else{
            d = da + 270;
        }
        //
        double x = pr1len * Math.cos(Math.toRadians(c)) + lielen * Math.cos(Math.toRadians(d - (90 - c)));
        double y = pr1len * Math.sin(Math.toRadians(c)) + lielen * Math.cos(Math.toRadians(d - (90 - c)));
        //
        xny.add(x);
        xny.add(y);
        //
        return xny;
    }
    //
    public static List<Double> lumina(double x, double y, double c, double d){
        //DEGREES(ATAN(G4/G3)) + DEGREES(ACOS(/(G1^2+(G3^2+G4^2)-G2^2)/(2*G1*SQRT(G3^2+G4^2))/))
        //degrees(asin(sin(acos((G1^2+G3^2+G4^2-G2^2)/(2*G1*sqrt(G3^2+G4^2))))*sqrt(G3^2+G4^2)/G2))
        /* g1 - pr1len
        g2 - lielen
        g3 - x
        g4 - y
         */
        List<Double> pnl = new ArrayList<>();
        //
        double cb = Math.toDegrees(Math.atan(y / x)) + Math.toDegrees(Math.acos((Math.pow(pr1len, 2) + (Math.pow(x, 2) + Math.pow(y, 2)) - Math.pow(lielen, 2)) / (2 * pr1len * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)))));
        double db = Math.toDegrees(Math.asin(Math.sin(Math.acos((Math.pow(pr1len, 2) + (Math.pow(x, 2) + Math.pow(y, 2)) - Math.pow(lielen, 2)) / (2 * pr1len * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2))))) * Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)) / lielen));
        //IF(G7-90>-1,G7-90,G7+270)
        if (db - 90 > -1){
            db = db - 90;
        }else{
            db = db + 270;
        }
        //
        double p = cb - c;
        double l = db - d;
        //
        double ps;
        double ls;
        //
        if (Math.abs(p) > Math.abs(l)){
            ps = 1.0;
            ls = l / p;
        } else {
            ps = p / l;
            ls = 1.0;
        }
        //
        if (p < -1){
            ps = ps * -1;
        }
        //
        if (l < -1){
            ls = ls * -1;
        }
        //
        pnl.add(ps);
        pnl.add(ls);
        //
        return pnl;
    }
    //
    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
