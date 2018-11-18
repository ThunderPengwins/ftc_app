package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Wall-G", group = "real")
public class Wall_G extends OmniAutoMode{
    //
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
    static final Double closed = .5;
    static final Double open = 0.1;
    //
    public void runOpMode(){
        //
        telInit("hardware");
        //<editor-fold desc="Hardware">
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
        //
        withoutEncoder();
        //</editor-fold>
        //
        //<editor-fold desc="DogeCV">
        telInit("DogeCV");
        //
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default setting
        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -10; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        //
        detector.enable(); // Start the detector!
        //</editor-fold>
        //
        telInit("gyro");
        initGyro();
        //
        telInit("Servos");
        releaseTheHounds.setPosition(0);
        flapper.setPosition(0);
        //
        telInit("complete");
        //
        waitForStartify();
        //
        lowerBot(2.0);
        //
        turnWithGyro(20, .3);
        withoutEncoder();
        //
        turn(.3);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while (!detector.getAligned() && opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("degrees", -angles.firstAngle);
            telemetry.update();
        }
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        turnWithGyro(10, -.4);
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
        telMove(position.toString());
        sleep(1000);
        //
        telMove("forward");
        moveToPosition(20, .2);
        //
        telMove("complete");
        //turn if needed
        if (position == -1){
            telMove("turn right");
            turnWithGyro(10, .3);
            //
            moveToPosition(8, .3);
            //
            turnWithGyro(35, .3);
        } else if(position == 1){
            turnWithGyro(15, -.3);
        }
        flapper.setPosition(1);
        sleep(2000);
        //
    }
    //
    public void followall(Integer distance){
        drive(.2);
        //
        while (jeep.getDistance(DistanceUnit.INCH) > distance){
            if (wall.getDistance(DistanceUnit.INCH) < 5){
                telMove("Too close!");
                right.setPower(right.getPower() - .01);
            } else if (wall.getDistance(DistanceUnit.INCH) > 8 || wall.getDistance(DistanceUnit.INCH) == DistanceUnit.infinity){
                telMove("Too far!");
                left.setPower(left.getPower() - .01);
            } else {
                telMove("Just Right");
                drive(.2);
            }
        }
        //
        drive(0);
    }
    //
    public void lowerBot(Double power){
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
}
