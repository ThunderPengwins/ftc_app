package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class Globulus extends OmniAutoMode{
    //<editor-fold desc="Hardware">
    private GoldAlignDetector detector;
    ModernRoboticsI2cRangeSensor jeep;
    DistanceSensor wall;
    DcMotor vertical;
    DigitalChannel down;
    DigitalChannel up;
    Servo latch;
    Servo flapper;
    Servo releaseTheHounds;
    //
    static final Double closed = .92;
    static final Double open = 0.5;
    //</editor-fold>
    //
    public void beginify(){
        //
        telInit("hardware");
        //<editor-fold desc="HardwareMap">
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        wall = hardwareMap.get(DistanceSensor.class, "wall");
        vertical = hardwareMap.dcMotor.get("vertical");//change in phones
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
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    public void lowerBot (Double power){
        //
        vertical.setPower(power);
        //
        while (!up.getState()) {
            telemetry.addData("up", up.getState());
            telemetry.update();
        }
        //
        vertical.setPower(0);
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
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -200; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

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
}