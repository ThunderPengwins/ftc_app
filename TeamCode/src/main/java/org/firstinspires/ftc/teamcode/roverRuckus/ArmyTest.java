package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ArmyTest", group = "test")
public class ArmyTest extends OmniMode {
    //
    static final double pr1len = 14.5;
    static final double lielen = 18;
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
    Servo hand;
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
        hand = hardwareMap.servo.get("hand");
        //</editor-fold>
        //
        double c;
        double d;
        //
        double xinput = 0;
        double yinput = 0;
        //
        double xdir;
        double ydir;
        //
        double highpower;
        //
        boolean stationary = false;
        int pposition = pr1vate.getCurrentPosition();
        int lposition = lieutenant.getCurrentPosition();
        //
        double powor;
        double lowpowor;
        double liepowor;
        double lowliepowor = .04163;
        double ppow = 0;
        double lpow = 0;
        //
        List<Double> cords = new ArrayList<>();
        List<Double> powers = new ArrayList<>();
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pr1vate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pr1vate.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        waitForStartify();
        //
        pr1vate.setPower(0);
        lieutenant.setPower(0);
        //
        while (opModeIsActive()) {
            //
            //<editor-fold desc="Find Coordinates">
            xinput = -gamepad2.left_stick_y;
            yinput = -gamepad2.right_stick_y;
            //
            xdir = filter(xinput);
            ydir = filter(yinput);
            //
            if (Math.abs(xinput) > Math.abs(yinput)){
                highpower = Math.abs(xinput);
            }else{
                highpower = Math.abs(yinput);
            }
            //
            c = confine(voltogres(cinput.getVoltage()) + 15);
            d = voltogres(dinput.getVoltage());
            //
            cords = xny(c, d);
            //</editor-fold>
            //
            powor = (.025 * (cords.get(0) - 11)) + .44;
            lowpowor = (.0035 * (cords.get(0) - 15.5));
            liepowor = (-.002 * (Math.abs(d))) + .68;
            //
            if (!(xdir == 0 && ydir == 0)){
                //<editor-fold desc="If Moving">
                //
                if (stationary){
                    stationary = false;
                }
                //
                powers = lumina(cords.get(0) + xdir, cords.get(1) + ydir, c, d);
                //l  fd
                if (powers.get(0) > 0){
                    ppow = powers.get(0) * .05 + powor * highpower;
                }else{
                    ppow = powers.get(0) * .05 + lowpowor * highpower;
                }
                //
                if (powers.get(1) > 0){
                    lpow = powers.get(1) * .05 * 5/6 + liepowor * highpower;
                } else{
                    lpow = powers.get(1) * .05 * 5/6 + lowliepowor * highpower;
                }
                //
                if (c > 129 && powers.get(0) > 0){
                    pr1vate.setPower(0);
                    lieutenant.setPower(0);
                }else {
                    pr1vate.setPower(ppow);
                    lieutenant.setPower(lpow);
                }
                //</editor-fold>
            }else{
                //<editor-fold desc="Hold position">
                //
                powers.clear();
                powers.add(0.0);
                powers.add(0.0);
                powers.add(c);
                powers.add(d);
                //
                if (!stationary){
                    pposition = pr1vate.getCurrentPosition();
                    lposition = lieutenant.getCurrentPosition();
                    //
                    stationary = true;
                }
                //
                corToZero(pposition, lposition);
                //</editor-fold>
            }
            //180-((B4-90)-(90-B3))
            hand.setPosition((250 - d - c) / 180);
            //130
            //
            if (gamepad2.dpad_up){
                dump();
            }
            //
            //<editor-fold desc="Telemetry">
            telemetry.addData("cinput", round(cinput.getVoltage(), 2) + ", degrees: " + round(c, 2) + ", new: " + round(powers.get(2), 2));
            telemetry.addData("dinput", round(dinput.getVoltage(), 2) + ", degrees: " + round(d, 2) + ", new: " + round(powers.get(3), 2));
            telemetry.addData("x", round(cords.get(0), 2) + ", y: " + round(cords.get(1), 2));
            telemetry.addData("new d", cords.get(2) + ", powor: " + powor);
            telemetry.addData("private's math", round(powers.get(0), 2) + ", power: " + round(ppow, 2) + ", position: "+ pr1vate.getCurrentPosition());
            telemetry.addData("lieutenant's math", round(powers.get(1), 2) + ", power: " + round(lpow, 2) + ", position: " + lieutenant.getCurrentPosition());
            telemetry.addData("stationary?", (xdir == 0 && ydir == 0) + ", X dir: " + xdir + ", Y dir: " + ydir);
            telemetry.addData("powor", powor + ", lowpowor: " + lowpowor + ", liepowor: " + liepowor);
            telemetry.addData("within range", (withinRange(voltogres(cinput.getVoltage()) + 15, 90, 7) && withinRange(voltogres(dinput.getVoltage()), 90, 15)));
            telemetry.update();
            //</editor-fold>
        }
    }
    //
    public double voltogres(double volts){
        int n;
        for (n = 0; n < voltage.size() - 1 && voltage.get(n) < volts; n++){}
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
        double y = pr1len * Math.sin(Math.toRadians(c)) + lielen * Math.sin(Math.toRadians(d - (90 - c)));
        //
        xny.add(x);
        xny.add(y);
        xny.add(d);
        //
        return xny;
    }
    //
    public List<Double> lumina(double x, double y, double c, double d){
        //DEGREES(ATAN(G4/G3)) + DEGREES(ACOS(/(G1^2+(G3^2+G4^2)-G2^2)/(2*G1*SQRT(G3^2+G4^2))/))
        //DEGREES(ACOS((G2^2+G1^2-(G3^2+G4^2))/(2*G1*G2)))
        /* g1 - pr1len
        g2 - lielen
        g3 - x
        g4 - y
         */
        //
        List<Double> pnl = new ArrayList<>();
        //((G1^2+(G3^2+G4^2)-G2^2)/(2*G1*SQRT(G3^2+G4^2)))
        //
        Double value1 = (Math.pow(pr1len, 2) + (Math.pow(x, 2) + Math.pow(y, 2)) - Math.pow(lielen, 2))  / (2 * pr1len * Math.hypot(x, y));
        Double value2 = (Math.pow(lielen, 2) + Math.pow(pr1len, 2) - (Math.pow(x, 2) + Math.pow(y, 2))) / (2 * pr1len * lielen);
        Double value3 = Math.atan(y / x);
        //
        if (-1 < value1 && value1 < 1 && -1 < value2 && value2 < 1){
            //
            //IF(G7-90>-1,G7-90,G7+270)
            //
            double cb = Math.toDegrees(Math.atan(y / x)) + Math.toDegrees(Math.acos((Math.pow(pr1len, 2) + (Math.pow(x, 2) + Math.pow(y, 2)) - Math.pow(lielen, 2)) / (2 * pr1len * Math.hypot(x, y))));
            double db = Math.toDegrees(Math.acos((Math.pow(lielen, 2) + Math.pow(pr1len, 2) - (Math.pow(x, 2) + Math.pow(y, 2))) / (2 * pr1len * lielen)));
            //=DEGREES(ACOS((G2^2+G1^2-(G3^2+G4^2))/(2*G1*G2)))
            //
            double p = cb - c;
            double l = db - d;
            //
            double ps;
            double ls;
            //
            if (Math.abs(p) > Math.abs(l)) {
                ps = 1.0;
                ls = Math.abs(l / p);
            } else {
                ps = Math.abs(p / l);
                ls = 1.0;
            }
            //
            if (p < 0) {
                ps = ps * -1;
            }
            //
            if (l < 0) {
                ls = ls * -1;
            }
            //
            pnl.add(ps);
            pnl.add(ls);
            //
            pnl.add(cb);
            pnl.add(db);
            //
            return pnl;
        }else{
            pnl.add(0.0);
            pnl.add(0.0);
            //
            pnl.add(c);
            pnl.add(d);
            //
            return pnl;
        }
    }
    //
    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(value);
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
    //
    public double confine (double input){
        if (input < 0){
            return 0;
        }else if (input > 180){
            return 180;
        }else{
            return input;
        }
    }
    //
    public double filter (double input){
        double output;
        //
        if (input < 0){
            output = -1;
        } else if (input > 0){
            output = 1;
        }else{
            output = 0;
        }
        return output;
    }
    //
    public void corToZero(double ppos, double lpos){
        //if (pr1vate.getCurrentPosition() > ppos){
            pr1vate.setPower(.15);
        /*}else{
            pr1vate.setPower(0);
        }*/
        //
        //if (lieutenant.getCurrentPosition() > lpos){
            lieutenant.setPower(.15);
        /*}else{
            lieutenant.setPower(0);
        }*/
    }
    //
    public void armToPosition (double c, double d, double cspeed, double dspeed){
        if (voltogres(cinput.getVoltage()) + 15 > c){
            cspeed = -cspeed;
        }
        //
        if (voltogres(dinput.getVoltage()) > d){
            dspeed = -dspeed;
        }
        //
        //pr1vate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lieutenant.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        pr1vate.setPower(cspeed);
        lieutenant.setPower(dspeed);
        //
        while (!(withinRange(voltogres(cinput.getVoltage()) + 15, c, 7) && withinRange(voltogres(dinput.getVoltage()), d, 15))){
            if (withinRange(voltogres(cinput.getVoltage()) + 15, c, 7)){
                pr1vate.setPower(0);
            }
            if (withinRange(voltogres(dinput.getVoltage()), d, 15)){
                lieutenant.setPower(0);
            }
        }
        //
        //pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
    }
    //
    public boolean withinRange (double input, double target, double error){
        if (input > target - error && input < target + error){
            return true;
        }else{
            return false;
        }
    }
    //
    public void dump(){
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        armToPosition(90, 180, .2, .2);
        //
        pr1vate.setPower(0);
        lieutenant.setPower(0);
        //
        while (!gamepad2.dpad_down){
            if (gamepad2.dpad_up){
                hand.setPosition(1);
            }else{
                hand.setPosition(0);
            }
        }
        //
        armToPosition(45, 100, .2, .2);
        //
        pr1vate.setPower(0);
        lieutenant.setPower(0);
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
    }
}