package org.firstinspires.ftc.teamcode.roverRuckus;

        import com.disnodeteam.dogecv.CameraViewDisplay;
        import com.disnodeteam.dogecv.DogeCV;
        import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//an ominous autonomous

@Autonomous(name = "Linus", group = "test")
public class Linus extends OmniAutoMode {
    //
    private GoldAlignDetector detector;
    //
    public void runOpMode(){
        //
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        //
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        //
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default setting
        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -10; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.constrainSize = 50;
        detector.constrainPosOffset = 75;

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        //
        detector.enable(); // Start the detector!
        //
        waitForStartify();
        //
        while (opModeIsActive() && !detector.getAligned()){
            telemetry.addData("Is Aligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("Is Constrained", detector.getConstrained());
            telemetry.addData("Y Pos" , detector.getY()); // Gold X position.
        }
    }
    //
}
