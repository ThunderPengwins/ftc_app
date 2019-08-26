package org.firstinspires.ftc.teamcode.roverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Created by Pilt on 12/14/17.
 */

public abstract class OmniAutoMode extends OmniMode{
    //
    double rotationInches;
    //
    ElapsedTime time = new ElapsedTime();
    //
    //<editor-fold desc="Init Vuforia">
    private static final String VUFORIA_KEY = "AbxR5+T/////AAAAGR1YlvU/6EDzrJvG5EfPnXSFutoBr1aCusr0K3pKqPuWTBQsUb0mv5irjoX2Xf/GFvAvHyw8v1GBYgHwE+hNTcNj05kw3juX+Ur4l3HNnp5SfXV/8fave0xB7yVYZ/LBDraNnYXiuT+D/5iGfQ99PVVao3LI4uGUOvL9+3vbPqtTXLowqFJX5uE7R/W4iLmNqHgTCSzWcm/J1CzwWuOPD252FDE9lutdDVRri17DBX0C/D4mt6BdI5CpxhG6ZR0tm6Zh2uvljnCK6N42V5x/kXd+UrBgyP43CBAACQqgP6MEvQylUD58U4PeTUWe9Q4o6Xrx9QEwlr8v+pmi9nevKnmE2CrPPwQePkDUqradHHnU";
    //Vuforia License Key
    //
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;// the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;// the height of th center of the target image from the floor
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    //
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
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
    VuforiaLocalizer vuforia;
    //</editor-fold>
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    ModernRoboticsI2cRangeSensor jeep;
    ModernRoboticsI2cRangeSensor wall;
    ModernRoboticsI2cRangeSensor leftwall;
    //
    //<editor-fold desc="Yay">
    abstract public void runOpMode();
    //
    static final double countify = 88;//41.5
    //</editor-fold>
    //
    //<editor-fold desc="Extraneous">
    //
    public void stopAndResetify(){
        resetEncoder();
        //
        toPosition();
    }
    //
    private void setSpeed(double speed){
        left.setPower(speed);
        right.setPower(speed);
    }
    //
    public void fancyStops(){
        while (likeToMoveIt() && threatened(3)){}
        //
        right.setPower(0);
        left.setPower(0);
    }
    //
    public void stops(){
        while (likeToMoveIt()){
            telemetry.addData("left", left.getCurrentPosition());
            telemetry.addData("right", right.getCurrentPosition());
            telemetry.update();
        }
        //
        right.setPower(0);
        left.setPower(0);
    }
    //</editor-fold>
    //
    //<editor-fold desc="Moving">
    public void moveToPosition(double inches, double speed){
        toPosition();
        //
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() + move);
        right.setTargetPosition(right.getCurrentPosition() + move);
        //
        setSpeed(speed);
        //
        stops();
    }
    //</editor-fold>
    //
    //<editor-fold desc="turning">
    public void turnRightToPosition(double inches, double speed){
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() + move);
        right.setTargetPosition(right.getCurrentPosition() - move);
        //
        setSpeed(speed);
    }
    //
    public void turnLeftToPosition (double inches, double speed){
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() - move);
        right.setTargetPosition(right.getCurrentPosition() + move);
        //
        setSpeed(speed);
    }
    //
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);//To 0 to 360
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175    To -180 t0 180
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            turn(0);//stop
        }
        //</editor-fold>
        //
        stopAndResetify();
    }
    //
    public void shimmyWithGyro(double degrees, double speedDirection, int motor){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        shimmyWithEncoder(speedDirection, motor);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        shimmyWithEncoder(speedDirection / 3, motor);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            turn(0);//stop
        }
        //</editor-fold>
        //
        stopAndResetify();
    }
    //
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    //
    public double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }
    //
    public void wiggleToPosition(double inches, double speed){
        toPosition();
        //
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() + move);
        right.setTargetPosition(right.getCurrentPosition() + move/2);
        //
        left.setPower(speed);
        right.setPower(speed/2);
        //
        stops();
    }
    //
    public void wiggleWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        withEncoder();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        right.setPower(-speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
//                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
//                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        right.setPower(speedDirection/3);//turn to second position
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
//                telemetry.update();
            }
        }else{
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
//                telemetry.update();
            }
        }
        right.setPower(0);//stop
        //</editor-fold>
        //
        stopAndResetify();
        //
    }
    //
    public boolean threatened(Integer distance){
        if (wall.getDistance(DistanceUnit.INCH) > 3){
            return true;
        }else{
            return false;
        }
    }

    public void inchesMoved(){
    }

    public void wallFollower(int wallInches, int movingRoations){
       while(movingRoations<(movingRoations+1)){
           if (wallInches <= jeep.getDistance(DistanceUnit.INCH)) {
               turnWithGyro(40, 0.5);
               drive(0.75);
           }
           else if (wallInches >= jeep.getDistance(DistanceUnit.INCH)) {
               turnWithGyro(-40, 0.5);
               drive(0.75);
           }
           movingRoations++;
           telemetry.addData("Wall Inches", jeep.getDistance(DistanceUnit.INCH));
           telemetry.update();
       }
    }
    //
    public void fwright(double inches, double speed, double wallDistance){
        toPosition();
        //
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() + move);
        right.setTargetPosition(right.getCurrentPosition() + move);
        //
        setSpeed(speed);
        //
        while (likeToMoveIt()){
            if (wall.getDistance(DistanceUnit.INCH) < wallDistance - 2){
                left.setPower((left.getPower()) - (.05));
                right.setPower(speed);
            }else if (wall.getDistance(DistanceUnit.INCH) > wallDistance + 2){
                right.setPower((right.getPower()) - (.05));
                left.setPower(speed);
            }else{
                left.setPower(speed);
                right.setPower(speed);
            }
        }
        //
        setSpeed(0);
    }
    //
    public void fwbackright(double inches, double speed, double wallDistance){
        toPosition();
        //
        int move = (int)(Math.round(inches*countify));
        //
        left.setTargetPosition(left.getCurrentPosition() + move);
        right.setTargetPosition(right.getCurrentPosition() + move);
        //
        setSpeed(speed);
        //
        while (likeToMoveIt()){
            if (withinRange(leftwall.getDistance(DistanceUnit.INCH), leftwall.getDistance(DistanceUnit.INCH), 0)){
                left.setPower(speed);
                right.setPower(speed);
            }else if (leftwall.getDistance(DistanceUnit.INCH) < wall.getDistance(DistanceUnit.INCH)){
                left.setPower((left.getPower()) - (.05));
                right.setPower(speed);
            }else{
                right.setPower((right.getPower()) - (.05));
                left.setPower(speed);
            }
        }
        //
        setSpeed(0);
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
}