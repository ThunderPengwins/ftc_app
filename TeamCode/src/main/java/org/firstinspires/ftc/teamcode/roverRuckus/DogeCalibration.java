package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous(name = "Doge Calibration", group = "real")
public class DogeCalibration extends Globulus {
    //
    public double result;
    //
    private GoldAlignDetector detector;
    //
    static final double target = 85;
    static final double pxperdg = 10;
    //
    public void runOpMode() {
        //
        waitForStartify();
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
        detector.alignPosOffset = -50; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        //
        detector.enable(); // Start the detector!
        //
        /*lowerBot(1.0);
        //
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
        //
        double difference = getAngle() - target;
        //
        double change = difference * pxperdg;*/
        //

        //
        String filename = "DogeCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, serialize(Double.toString(detector.getY())));
    }
    //
    public String serialize(String input) {
        return SimpleGson.getInstance().toJson(input);
    }
    //
    /*public String deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data,json);}*/
}
