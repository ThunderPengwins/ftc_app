package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Wall-E", group = "real")
public class Wall_E extends OmniAutoMode{
    //
    private GoldAlignDetector detector;
    ModernRoboticsI2cRangeSensor jeep;
    DistanceSensor wall;
    //
    public void runOpMode(){
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        jeep = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "jeep");
        wall = hardwareMap.get(DistanceSensor.class, "wall");
        //
        configureMotors();
        //
        withoutEncoder();
        //
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
        //
        telInit("gyro");
        initGyro();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //
        telInit("complete");
        //
        waitForStartify();
        //
        //Insert lowering of arm
        //
        turn(.2);
        time.reset();
        //Turn towards mineral
        telMove("Looking for mineral");
        while (!detector.getAligned()){}
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //Check position of mineral
        if (-angles.firstAngle < 80){
            position = -1;
        }else if(80 <= -angles.firstAngle && -angles.firstAngle < 100){
            position = 0;
        }else{
            position = 1;
        }
        //
        telMove("waiting");
        waitify(1000);
        //
        telMove("forward");
        drive(.2);
        //
        while (detector.isFound()){}
        drive(0);
        //
        telMove("complete");
        //turn if needed
        if (position == -1){
            telMove("turn right");
            turnWithGyro(20, .1);
            toPosition();
            telMove("more stuff!");
            moveToPosition(5, .1);
        } else if(position == 1){
            telMove("turn left");
            turnWithGyro(20, -.1);
            toPosition();
            telMove("more stuff!");
            moveToPosition(5, .1);
        }
        //move to depot
        moveToPosition(36, .1);
        //back away from the cube
        moveToPosition(-2, .1);
        //turn towards wall
        turnWithGyro(45 + ((position) * 35), .1);//turn different based on position
        //
        /*
        insert deposit marker
         */
        //drive to wall
        drive(.1);
        while (jeep.getDistance(DistanceUnit.INCH) > 5){}
        drive(0);
        //
        turnWithGyro(90, .1);
        //
        followall(54);
        //
        /*
        Break the plane
         */
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
}
