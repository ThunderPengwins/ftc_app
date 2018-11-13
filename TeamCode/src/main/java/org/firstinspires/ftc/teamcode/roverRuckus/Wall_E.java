package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Wall-E", group = "real")
public class Wall_E extends OmniAutoMode{
    //
    private GoldAlignDetector detector;
    //
    public void runOpMode(){
        //
        Double first = 1234.5678;
        Double second = 2345.6789;
        Double third = 3456.7890;
        //
        telInit("hardware");
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //
        configureMotors();
        //
        toPosition();
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
        //
        telInit("complete");
        //
        waitForStartify();
        //
        //Insert lowering of arm
        //
        turn(.2);
        time.reset();
        //
        telMove("Looking for mineral");
        while (!detector.getAligned()){}
        turn(0);
        Double stoptime = time.milliseconds();
        Integer position;
        //
        if (first - 300 < stoptime && stoptime < first + 300){
            position = 0;
        }else if(second - 300 < stoptime && stoptime < second + 300){
            position = 1;
        }else{
            position = 2;
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
    }
    //
}
