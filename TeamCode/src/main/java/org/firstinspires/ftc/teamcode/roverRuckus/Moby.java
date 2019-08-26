package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Eric
 */

@TeleOp (name = "Moby", group = "real")
public class Moby extends OmniMode {
    //<editor-fold desc="hardware">
    /**
     * <h1>Hardware</h>
     * <p>Here is where we create any piece of hardware that
     * wasn't already created in the class above, OmniMode.
     * Later on, we do the hardware map, but that can't be done
     * until we say:</p>
     * @code runOpMode(){}
     */
    ModernRoboticsI2cRangeSensor jeep;
    DcMotor verticala;
    DcMotor verticalb;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
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
    //</editor-fold>
    //
    static final Double closed = .0;
    static final Double open = 1.0;
    //
    public void runOpMode() {
        //
        //<editor-fold desc="Init">
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        verticala = hardwareMap.dcMotor.get("verticala");
        verticalb = hardwareMap.dcMotor.get("verticalb");
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        pr1vate = hardwareMap.dcMotor.get("pr1vate");//change in phones
        lieutenant = hardwareMap.dcMotor.get("lieutenant");
        cinput = hardwareMap.analogInput.get("cinput");
        dinput = hardwareMap.analogInput.get("dinput");
        hand = hardwareMap.servo.get("hand");
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pr1vate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pr1vate.setDirection(DcMotorSimple.Direction.REVERSE);
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
        //</editor-fold
        //
        //<editor-fold desc="Variables">
        //
        boolean fullpower = false;
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
        double lowliepowor = .041634;
        double ppow = 0;
        double lpow = 0;
        //
        List<Double> cords = new ArrayList<>();
        List<Double> powers = new ArrayList<>();
        //
        Integer direction = 0;//lifter direction
        Double rightPower = .5;//right motor power
        Double leftPower = .5;//left motor power
        Integer front = 1;//front side of robot
        Integer ready = 1;
        //
        configureMotors();
        telInit("complete");
        //</editor-fold>
        //
        waitForStartify();
        //
        while (opModeIsActive()) {
            //
            left.setPower(-gamepad1.left_stick_y * leftPower * front);
            right.setPower(-gamepad1.right_stick_y * rightPower * front);
            //
            //<editor-fold desc="Power Levels">
            //
            if (gamepad1.dpad_up){
                fullpower = true;
            }else if(gamepad1.dpad_down){
                fullpower = false;
            }
            //
            if (gamepad1.right_trigger > 0) {
                rightPower = .2;
                telemetry.addData("Trigger", "Right");
            } else if (gamepad1.right_bumper) {
                rightPower = .05;
                telemetry.addData("Bumper", "Right");
            } else {
                if (isTurning(-gamepad1.left_stick_y, -gamepad1.right_stick_y)){
                    rightPower = 1.0;
                }else {
                    if(fullpower){
                        rightPower = 1.0;
                    }else {
                        rightPower = .5;
                    }
                }
                telemetry.addData("No action", "Right");
            }
            //
            if (gamepad1.left_trigger > 0) {
                leftPower = .2;
                telemetry.addData("Trigger", "Left");
            } else if (gamepad1.left_bumper) {
                leftPower = .05;
                telemetry.addData("Bumper", "Left");
            } else {
                if (isTurning(-gamepad1.left_stick_y, -gamepad1.right_stick_y)) {
                    leftPower = 1.0;
                }else {
                    if(fullpower){
                        leftPower = 1.0;
                    }else {
                        leftPower = .5;
                    }
                    telemetry.addData("No action", "Left");
                }
            }
            //</editor-fold>
            //
            //<editor-fold desc="Lifter">
            if (!((up.getState() && direction == 1) || (down.getState() && direction == -1)) && !(gamepad2.right_trigger > 0) && !(direction == 0)) {
                verticala.setPower(0.5 * direction);
                verticalb.setPower(0.5 * direction);
            } else {
                verticala.setPower(0);
                verticalb.setPower(0);
                direction = 0;
            }
            //
            if (gamepad2.y){
                direction = 1;
            } else if (gamepad2.x){
                direction = -1;
            }
            //</editor-fold>
            //
            //<editor-fold desc="Latch">
            if (gamepad2.a) {
                latch.setPosition(closed);
            } else if (gamepad2.b) {
                latch.setPosition(open);
            }
            //</editor-fold>
            //
            //<editor-fold desc="Front">
            if (gamepad1.b) {
                if (front == 1 && ready == 1) {
                    front = -1;
                } else if (front == -1 && ready == 1) {
                    front = 1;
                }
                ready = 0;
            } else {
                ready = 1;
            }
            //</editor-fold>
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
            powor = (.025 * (cords.get(0) - 11)) + .46;
            lowpowor = (.0035 * (cords.get(0) - 15.5));
            liepowor = (-.002 * (Math.abs(d))) + .7;
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
            if (gamepad2.dpad_up){
                dump();
            }
            if (gamepad2.dpad_down){
                armToPosition(60, 100, .2, .2);
            }
            //
            if (gamepad2.dpad_left){
                armToPosition(120, 50, .2, .2);
            }
            //
            //<editor-fold desc="Telemetry">
            telemetry.addData("ready?", ready);
            telemetry.addData("front", front);
            telemetry.addData("distance", jeep.getDistance(DistanceUnit.INCH));
            telemetry.addData("up", up.getState());
            telemetry.addData("down", down.getState());
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
        //
        if (voltogres(cinput.getVoltage()) + 15 > c){
            cspeed = -cspeed;
        }
        //
        if (voltogres(dinput.getVoltage()) > d){
            dspeed = -dspeed;
        }
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            if (gamepad2.dpad_right){
                return;
            }
        }
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        List<Double> cords = new ArrayList<>();
        //
        double c = confine(voltogres(cinput.getVoltage()) + 15);
        double d = voltogres(dinput.getVoltage());
        //
        cords = xny(c, d);
        //
        double lowliepowor = .041634;
        double powor = (.025 * (cords.get(0) - 11)) + .44;
        double lowpowor = (.0035 * (cords.get(0) - 15.5));
        double liepowor = (-.002 * (Math.abs(d))) + .7;
        //
        hand.setPosition(0);
        //
        while (cords.get(0) > 8){
            c = confine(voltogres(cinput.getVoltage()) + 15);
            d = voltogres(dinput.getVoltage());
            //
            cords = xny(c, d);
            //
            lowliepowor = .041634;
            powor = (.025 * (cords.get(0) - 11)) + .44;
            lowpowor = (.0035 * (cords.get(0) - 15.5));
            liepowor = (-.002 * (Math.abs(d))) + .7;
            //
            pr1vate.setPower(.025 + powor);
            if (c > 180){
                lieutenant.setPower(.025 * 5/6 + lowliepowor);
            }else {
                lieutenant.setPower(.025 * 5 / 6 + liepowor);
            }
            //
            if (gamepad2.dpad_down){
                armToPosition(45, 100, .2, .2);
                pr1vate.setPower(0);
                lieutenant.setPower(0);
                return;
            }
        }
        //
        pr1vate.setPower(0);
        lieutenant.setPower(0);
        //
        lieutenant.setPower(.5);
        while (!withinRange(d, 180, 7)){
            d = voltogres(dinput.getVoltage());
            if (gamepad2.dpad_down){
                armToPosition(45, 100, .2, .2);
                return;
            }
        }
        //
        lieutenant.setPower(0);
        //
        pr1vate.setPower(.3);
        while (!withinRange(c, 120, 7)){
            c = confine(voltogres(cinput.getVoltage()) + 15);
            if (gamepad2.dpad_down){
                armToPosition(45, 100, .2, .2);
                return;
            }
        }
        pr1vate.setPower(0);
        //
        while (!gamepad2.dpad_down){
            if (gamepad2.dpad_up){
                hand.setPosition(1);
            }else{
                hand.setPosition(0);
            }
            if (gamepad2.dpad_down){
                armToPosition(45, 100, .2, .2);
                return;
            }
        }
        //
        armToPosition(45, 100, .2, .2);
    }
    //
    public boolean isTurning(double left, double right){
        if (left * right < 0){
            return true;
        }else{
            return false;
        }
    }
}