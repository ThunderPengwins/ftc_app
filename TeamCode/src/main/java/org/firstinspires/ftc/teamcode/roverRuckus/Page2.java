package org.firstinspires.ftc.teamcode.roverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

//an ominous autonomous

@Autonomous(name = "Page 2?", group = "test")
public class Page2 extends OmniAutoMode {
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
        String filename = "DogeCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        String use = ReadWriteFile.readFile(file);
        //
        while (opModeIsActive() && !detector.getAligned()){
            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos" , detector.getY()); // Gold X position.
            telemetry.addData("read", use);
            telemetry.update();
        }
    }
    //
    public String trim (String input){
        String result;
        //
        char[] chars = input.toCharArray();
        //
        List<Character> characters = new ArrayList<>();
        //
        for (Character a : chars){
            characters.add(a);
        }
        //
        characters.remove(0);
        characters.remove(characters.size() - 1);
        //
        Character[] newone = characters.toArray(new Character[0]);
        //
        StringBuilder sb = new StringBuilder();
        //
        for (Character a : newone){
            sb.append(a);
        }
        //
        result = sb.toString();
        //
        return result;
    }
}
