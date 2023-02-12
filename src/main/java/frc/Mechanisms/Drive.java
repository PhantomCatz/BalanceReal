package frc.Mechanisms;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class Drive
{
    public Boolean startDriving = false;

    final double GEAR_RATIO = 1.0/6.75;
    final double TIME_DELTA = 0.05;

    final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    final double DRVTRAIN_WHEEL_DIAMETER                  = 4.0;
    final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);
    final double DRVTRAIN_ENC_COUNTS_TO_INCH              = GEAR_RATIO * DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV;

    private double STOP_DISTANCE = 0.5;
    private double MIN_POWER = 0.1;
    private double ERROR_GAIN = 0.02;
    private double RATE_GAIN = 0.01;

    private double distanceOffset = 0.0;
    private double distanceRemain = 0.0;
    private double targetPower = 0.0;
    private double turnPower = 0.0;
    private double angleOffset = 0.0; //change for the final code EL 2/4
    private double currentAngle = 0.0;
    private double currentError = 0.0;
    private double prevError = 0.0;
    private double time = 0.0;
    private double prevTime = -1.0; // no big initial rate
    private double errorRate = 0;
    private Boolean backwards;

    private Timer timer = new Timer();

    private CatzLog data;

    public double Clamp(double min, double in, double max)
    {
        if(in > max)
        {
            return max;
        }
        else if(in < min)
        {
            return min;
        }
        else
        {
            return in;
        }
    }

    public Drive() {}
                                                                                    //sets wheel angle
    public void DriveStraight(double distance, double decelDistance, double maxSpeed, double wheelPos, double maxTime){ 
        startDriving = true;

        if(distance < 0){
            backwards = true;
        }
        else{
            backwards = false;
        }

        distanceOffset = Robot.drivetrain.getAveragePosition();
        angleOffset = Robot.navX.getAngle(); // change for final 2/4 EL
        distanceRemain = distance;

        timer.reset();
        timer.start();

        while(Math.abs(distanceRemain) >= STOP_DISTANCE && startDriving && time < maxTime){
            time = timer.get();
            currentAngle = Robot.navX.getAngle();
            currentError = angleOffset - currentAngle;
            errorRate = (currentError - prevError) / (time - prevTime);

            distanceRemain = distance + (Robot.drivetrain.getAveragePosition() - distanceOffset) * DRVTRAIN_ENC_COUNTS_TO_INCH;
            targetPower = Clamp(-1.0, distanceRemain / distance / decelDistance, 1.0) * maxSpeed;
            turnPower = Clamp(-1.0, ERROR_GAIN * currentError + RATE_GAIN * errorRate, 1.0); //"-" in front of Error and Rate
            
            if(Math.abs(targetPower) < MIN_POWER){
                targetPower = MIN_POWER * Math.signum(targetPower);
            }

            if(backwards){
                turnPower = -turnPower;
            }

            Robot.drivetrain.translateTurn(wheelPos, targetPower, turnPower);
            Timer.delay(TIME_DELTA);

            prevTime = time;
            prevError = currentError;

            if(Robot.dataCollection.logDataID == DataCollection.LOG_ID_DRIVE && Robot.dataCollection.logDataValues){
                data = new CatzLog(Robot.currentTime.get(), distanceRemain, Robot.drivetrain.getAveragePosition(), targetPower, currentError, currentAngle, errorRate, turnPower, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
                Robot.dataCollection.logData.add(data);
            }
        }

        Robot.drivetrain.autoDrive(0);

        startDriving = false;
    }

    public void StopDriving()
    {
        startDriving = false;
    }
}