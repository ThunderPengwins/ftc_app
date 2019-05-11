package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class Globulus extends OmniAutoMode{
    //<editor-fold desc="Hardware">
    private GoldAlignDetector detector;
    DcMotor verticala;
    DcMotor verticalb;
    DcMotor pr1vate;
    DcMotor lieutenant;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    Servo flapper;
    Servo releaseTheHounds;
    AnalogInput cinput;
    AnalogInput dinput;
    //
    static final Double open = 1.0;
    static final Double closed = 0.0;
    static final Integer calibrationInput = -40;//-15
    //</editor-fold>
    //
    public void beginify(){
        //
        telInit("hardware");
        //<editor-fold desc="HardwareMap">
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        wall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wall");
        verticala = hardwareMap.dcMotor.get("verticala");//change in phones
        verticalb = hardwareMap.dcMotor.get("verticalb");//change in phones
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        releaseTheHounds = hardwareMap.servo.get("release");
        flapper = hardwareMap.servo.get("flapper");
        //</editor-fold>
        //
        //<editor-fold desc="Configure Motors">
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        verticala.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticala.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticala.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalb.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        configureMotors();
        //</editor-fold>
        //
        //<editor-fold desc="Gyro and Servos">
        telInit("gyro");
        initGyro();
        //
        telInit("Servos");
        releaseTheHounds.setPosition(0);
        flapper.setPosition(0);
        //
        telInit("complete");
        //
        sleep(1000);
        //</editor-fold>
    }
    //
    public void newbeginify(){
        //
        telInit("hardware");
        //<editor-fold desc="HardwareMap">
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        wall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "wall");
        leftwall = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftwall");
        verticala = hardwareMap.dcMotor.get("verticala");//change in phones
        verticalb = hardwareMap.dcMotor.get("verticalb");//change in phones
        pr1vate = hardwareMap.dcMotor.get("pr1vate");
        lieutenant = hardwareMap.dcMotor.get("lieutenant");
        cinput = hardwareMap.analogInput.get("cinput");
        dinput = hardwareMap.analogInput.get("dinput");
        down = hardwareMap.get(DigitalChannel.class, "down");
        up = hardwareMap.get(DigitalChannel.class, "up");
        latch = hardwareMap.servo.get("latch");
        releaseTheHounds = hardwareMap.servo.get("release");
        //</editor-fold>
        //
        //<editor-fold desc="Configure Motors">
        down.setMode(DigitalChannel.Mode.INPUT);
        up.setMode(DigitalChannel.Mode.INPUT);
        //
        verticala.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticala.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticala.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalb.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pr1vate.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        configureMotors();
        //</editor-fold>
        //
        //<editor-fold desc="Gyro and Servos">
        telInit("gyro");
        initGyro();
        //
        telInit("Servos");
        releaseTheHounds.setPosition(0);
        //
        telInit("complete");
        //
        sleep(1000);
        //</editor-fold>
    }
    //
    public Integer sarker() {
        beginify();
        //
        waitForStartify();
        //
        dogeCV();
        //
        lowerBot(2.0);
        //
        sleep(1000);
        //
        //<editor-fold desc="Turn to Mineral">
        turnWithGyro(30, .3);
        //
        turnWithEncoder(.2);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while ((!detector.getAligned() || detector.getY() > 200) && opModeIsActive() && getAngle() < 120) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("height", detector.getY());
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (getAngle() < 75) {
            position = -1;
        } else if (75 <= getAngle() && getAngle() < 100) {
            position = 0;
        } else {
            position = 1;
        }
        //
        telMove(position.toString());
        //</editor-fold>
        //Go to mineral
        //<editor-fold desc="Put Mineral in Depot">
        telMove("to mineral");
        //Push into depot
        if (position == -1) {
            moveToPosition(27, .5);
            telMove("turn right");
            turnWithGyro(20, .4);
            telMove("more stuff!");
            moveToPosition(14, .5);
            //
            turnWithGyro(55, .4);
            //
            drive(.4);
            //
            while (jeep.getDistance(DistanceUnit.INCH) > 10){}
            //
            drive(0);
        } else if (position == 1) {
            moveToPosition(33, .5);
            //
            turnWithGyro(70, -.4);
            //
            drive(.5);
            //
            while (jeep.getDistance(DistanceUnit.INCH) > 8){}
            //
            drive(0);
        } else {
            moveToPosition(45, .3);
        }
        //back away from the cube
        moveToPosition(-2, .3);
        //</editor-fold>
        //turn towards wall
        turnWithGyro(45 + ((-position) * 35), -.3);//turn different based on position
        //
        releaseTheHounds.setPosition(.4);
        sleep(1000);
        //
        return position;
    }
    //
    public Integer sater(){
        beginify();
        //
        waitForStartify();
        //
        dogeCV();
        //
        //<editor-fold desc="Turn to Mineral">
        lowerBot(2.0);
        //
        sleep(1000);
        //
        turnWithGyro(30, .3);
        //
        turn(.25);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while ((!detector.getAligned() || detector.getY() > 200)  && opModeIsActive() && getAngle() < 160){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("degrees", -angles.firstAngle);
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (getAngle() < 80){
            position = -1;
        }else if(80 <= getAngle() && getAngle() < 100){
            position = 0;
        }else{
            position = 1;
        }
        //
        telemetry.addData("position", position);
        telemetry.addData("degrees", getAngle());
        sleep(500);
        //</editor-fold>
        //
        telMove("forward");
        moveToPosition(18, .4);
        //
        return position;
    }
    //
    public void tarker(){
        //<editor-fold desc="The beginning stuff">
        newbeginify();
        //
        waitForStartify();
        //
        dogeCV();
        //
        lowerBot(2.0);
        //
        lowerLifter(2.0, 1);
        //</editor-fold>
        //
        //<editor-fold desc="Turn to Mineral">
        turnWithGyro(20, .3);
        //
        turnWithEncoder(.2);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while ((!detector.getConstrained()) && opModeIsActive() && getAngle() < 120) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("height", detector.getY());
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (getAngle() < 75) {
            position = -1;
        } else if (75 <= getAngle() && getAngle() < 100) {
            position = 0;
        } else {
            position = 1;
        }
        //</editor-fold>
        //
        //<editor-fold desc="Put Mineral in Depot">
        telMove("to mineral");
        //Push into depot
        if (position == -1) {
            //
            moveToPosition(14, .5);
            turveRightByPoint(0.0, 25.0, 25.0, .8);
            //
            goToWall(6, .3);
            //
            turveBackRightByPoint(3.0, 2.0, .9, .8);
            //
            goToWall(6, .3);
            //
            releaseTheHounds.setPosition(1);
            sleep(1000);
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(90, -.4);
            //
            fwright(60, .3, 7);
            //
            armToPosition(60, 70, .5, .5);
            //
        } else if (position == 0) {
            //
            moveToPosition(47, .5);
            //
            turnWithGyro(60, -.4);
            //
            goToWall(6, .3);
            //
            releaseTheHounds.setPosition(1);
            sleep(1000);
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(65, -.4);
            //
            fwright(70, .3, 7);
            //
            armToPosition(60, 60, .5, .5);
            //
        } else {
            //
            moveToPosition(14, .5);
            //
            turveLeftByPoint(-5.0, 12.0, 3.0, .8);
            //
            goToWall(6, .3);
            //
            releaseTheHounds.setPosition(1);
            sleep(1000);
            releaseTheHounds.setPosition(0);
            //
            turnWithGyro(65, -.4);
            //
            fwright(70, .3, 7);
            //
            armToPosition(60, 60, .5, .5);
            //
        }
        //</editor-fold>
    }
    //
    public void tater(){
        //<editor-fold desc="The beginning stuff">
        newbeginify();
        //
        waitForStartify();
        //
        dogeCV();
        //
        lowerBot(2.0);
        //
        lowerLifter(2.0, 1);
        //</editor-fold>
        //
        //<editor-fold desc="Turn to Mineral">
        turnWithGyro(20, .3);
        //
        turnWithEncoder(.2);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while ((!detector.getConstrained()) && opModeIsActive() && getAngle() < 120) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("height", detector.getY());
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (getAngle() < 75) {
            position = -1;
        } else if (75 <= getAngle() && getAngle() < 100) {
            position = 0;
        } else {
            position = 1;
        }
        //</editor-fold>
        //
        //<editor-fold desc="Put Marker in Depot">
        telMove("to mineral");
        //Push into depot
        if (position == -1) {
            //
            moveToPosition(22, .5);
            //
            turnWithGyro(30, -.4);
            //
            turveLeftByPoint(5.0, 15.0, 3.0, .8);
            //
            fwright(50, .4, 7);
            //
            turnWithGyro(20, -.4);
            //
            goToWall(20, .4);
            //
            releaseTheHounds.setPosition(1);
            sleep(2000);
            releaseTheHounds.setPosition(0);
            sleep(1000);
            /*
            turnWithGyro(10, .4);
            //
            fwbackright(-40, -.4, 6);
            //
            fwbackright(-30, -.4, 8);
            //
            lieutenantBack(180);*/
        } else if (position == 0) {
            //
            moveToPosition(22, .5);
            //
            turveBackRightByPoint(10.0, 5.0, 2.0, .8);
            //
            moveToPosition(33, .5);
            //
            turveLeftByPoint(5.0, 15.0, 3.0, .8);
            //
            fwright(40, .4, 7);
            //
            goToWall(20, .4);
            //
            turnWithGyro(20,-.4);
            //
            releaseTheHounds.setPosition(1);
            sleep(2000);
            releaseTheHounds.setPosition(0);
            sleep(1000);
            //
        } else {
            //
            moveToPosition(22, .5);
            //
            turveBackRightByPoint(10.0, 5.0, 2.0, .8);
            //
            turveLeftByPoint(4.0, 10.0, 1.1, .8);
            //
            moveToPosition(21, .5);
            //
            turveLeftByPoint(5.0, 15.0, 3.0, .8);
            //
            fwright(30, .4, 7);
            //
            goToWall(20, .4);
            //
            turnWithGyro(20,-.4);
            //
            releaseTheHounds.setPosition(1);
            sleep(2000);
            releaseTheHounds.setPosition(0);
            sleep(1000);
        }
        //</editor-fold>
    }
    //
    public void tiny(){
        //<editor-fold desc="The beginning stuff">
        newbeginify();
        //
        waitForStartify();
        //
        dogeCV();
        //
        lowerBot(2.0);
        //
        lowerLifter(2.0, 1);
        //</editor-fold>
        //
        //<editor-fold desc="Turn to Mineral">
        turnWithGyro(20, .3);
        //
        turnWithEncoder(.2);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while ((!detector.getConstrained()) && opModeIsActive() && getAngle() < 120) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("height", detector.getY());
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (getAngle() < 75) {
            position = -1;
        } else if (75 <= getAngle() && getAngle() < 100) {
            position = 0;
        } else {
            position = 1;
        }
        //</editor-fold>
        //
        moveToPosition(22, .5);
    }
    //
    public void lowerBot (Double power){
        //
        verticala.setPower(power);
        verticalb.setPower(power);
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
        }
    //
    public void followall (Integer distance){
        drive(-.3);
        //
        while (jeep.getDistance(DistanceUnit.INCH) < distance) {
            if (wall.getDistance(DistanceUnit.INCH) < 5) {
                telMove("Too close!");
                right.setPower(right.getPower() + .01);
            } else if (wall.getDistance(DistanceUnit.INCH) > 8 || wall.getDistance(DistanceUnit.INCH) == DistanceUnit.infinity) {
                telMove("Too far!");
                left.setPower(left.getPower() + .01);
            } else {
                telMove("Just Right");
                drive(-.3);
            }
        }
        //
        drive(0);
    }
    //
    public void dogeCV (){
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default setting
        // Optional tuning
        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        Integer compound = -200 + calibrationInput;
        detector.alignPosOffset = compound; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.constrainPosOffset = 100;
        detector.constrainSize = 250;
        //
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        //
        detector.enable(); // Start the detector!
        //
    }
    //
    public void followGyro (Integer distance){
        drive(-.3);
        //
        while (jeep.getDistance(DistanceUnit.INCH) < distance) {
            if (getAngle() < 42) {
                telMove("Too close!");
                left.setPower(left.getPower() + .01);
            } else if (getAngle() > 48) {
                telMove("Too far!");
                right.setPower(right.getPower() + .01);
            } else {
                telMove("Just Right");
                drive(-.3);
            }
        }
        //
        drive(0);
    }
    //
    public void turveLeftByPoint(Double x, Double y, Double clearance, Double speed){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        Double leftMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double rightMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor * countify));
        int leftd = (int)(Math.round(leftMotor * countify));
        //
        telemetry.addData("left motor", leftMotor + ", " + leftd);
        telemetry.addData("right motor", rightMotor + ", " + rightd);
        telemetry.update();
        //
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(right.getCurrentPosition() + rightd);
        left.setTargetPosition(left.getCurrentPosition() + leftd);
        //
        right.setPower(speed);
        left.setPower((leftMotor / rightMotor) * speed);
        //
        while (right.isBusy()){
            telemetry.addData("left motor", leftMotor);
            telemetry.addData("right motor", rightMotor);
            telemetry.addData("left power", ((leftMotor / rightMotor) * .4) + ", reality: " + left.getPower());
            telemetry.addData("right power", .4 + ", reality: " + right.getPower());
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
        //
    }
    //
    public void turveRightByPoint(Double x, Double y, Double clearance, Double speed){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        Double rightMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double leftMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor * countify));
        int leftd = (int)(Math.round(leftMotor * countify));
        //
        telemetry.addData("left motor", leftMotor + ", " + leftd);
        telemetry.addData("right motor", rightMotor + ", " + rightd);
        telemetry.update();
        //
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(right.getCurrentPosition() + rightd);
        left.setTargetPosition(left.getCurrentPosition() + leftd);
        //
        left.setPower(speed);
        right.setPower((rightMotor / leftMotor) * speed);
        //
        while (right.isBusy()){
            telemetry.addData("left motor", leftMotor);
            telemetry.addData("right motor", rightMotor);
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
        //
    }
    //
    public void turveBackRightByPoint(Double x, Double y, Double clearance, Double speed){
        //
        Double width = (Math.hypot(x, y));
        Double radius = (clearance / 2) + (Math.pow(width, 2) / (8 * clearance));
        telemetry.addData("width", width);
        telemetry.addData("radius", radius);
        telemetry.update();
        //
        Double rightMotor = 2 * Math.PI * radius * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        Double leftMotor = 2 * Math.PI * (radius + 16) * ((2 * Math.toDegrees(Math.asin((width / 2) / radius))) / 360);
        //
        int rightd = (int)(Math.round(rightMotor * countify));
        int leftd = (int)(Math.round(leftMotor * countify));
        //
        telemetry.addData("left motor", leftMotor + ", " + leftd);
        telemetry.addData("right motor", rightMotor + ", " + rightd);
        telemetry.update();
        //
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        right.setTargetPosition(right.getCurrentPosition() - rightd);
        left.setTargetPosition(left.getCurrentPosition() - leftd);
        //
        left.setPower(-speed);
        right.setPower((rightMotor / leftMotor) * -speed);
        //
        while (right.isBusy()){
            telemetry.addData("left motor", leftMotor);
            telemetry.addData("right motor", rightMotor);
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
    }
    //
    public void lowerLifter (Double power, Integer seconds){
        //
        verticala.setPower(-power);
        verticalb.setPower(-power);
        //
        sleep(seconds * 1000);
        //
        verticala.setPower(0);
        verticalb.setPower(0);
    }
    //
    public double voltogres(double volts){
        int n;
        for (n = 0; n < voltage.size() - 1 && voltage.get(n) < volts; n++){}
        //
        return ((n * 15) / (voltage.get(n))) * (volts);
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
        }
        //
        sleep(2000);
        //
        pr1vate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lieutenant.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
    }
    //
    public boolean withinRange (double input, double target, double error){
        if (input > target - error || input < target + error){
            return true;
        }else{
            return false;
        }
    }
    //
    public void goToWall (double distance, double speed){
        //
        drive(speed);
        //
        while (jeep.getDistance(DistanceUnit.INCH) > distance){}
        //
        drive(0);
    }
    //
    public void lieutenantBack(double degrees){
        double d = voltogres(dinput.getVoltage());
        //
        lieutenant.setPower(.25);
        while (!withinRange(d, degrees, 7)){
            d = voltogres(dinput.getVoltage());
        }
        sleep(2000);
        //
        lieutenant.setPower(0);
    }
    //
}