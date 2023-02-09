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
    final double DRVTRAIN_WHEEL_RADIUS                    = 2.0;
    final double DRVTRAIN_WHEEL_CIRCUMFERENCE             = (2.0 * Math.PI * DRVTRAIN_WHEEL_RADIUS);
    final double DRVTRAIN_ENC_COUNTS_TO_INCH              = GEAR_RATIO * DRVTRAIN_WHEEL_CIRCUMFERENCE / TALONFX_INTEGRATED_ENC_CNTS_PER_REV;

    private double STOP_DISTANCE = 0.5;
    private double MIN_POWER = 0.1;
    private double ERROR_GAIN = 0.1;
    private double RATE_GAIN = 0.0;

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

    public void DriveStraight(double distance, double decelDistance, double maxSpeed, double maxTime){ 
        System.out.println("drive");

        startDriving = true;

        distanceOffset = Robot.drivetrain.getAveragePosition();
        angleOffset = Robot.navX.getAngle(); // change for final 2/4 EL
        distanceRemain = distance;

        timer.reset();
        timer.start();

        while(distanceRemain >= STOP_DISTANCE && startDriving && time < maxTime){
            time = timer.get();
            currentAngle = Robot.navX.getAngle();
            currentError = angleOffset - currentAngle;
            errorRate = (currentError - prevError) / (time - prevTime);

            distanceRemain = distance + (Robot.drivetrain.getAveragePosition() - distanceOffset) * DRVTRAIN_ENC_COUNTS_TO_INCH;
            targetPower = Clamp(MIN_POWER / Math.abs(maxSpeed), distanceRemain / distance / decelDistance, 1) * maxSpeed;
            turnPower = Clamp(-1.0, ERROR_GAIN * currentError + RATE_GAIN * errorRate, 1.0);
            Robot.drivetrain.translateTurn(0, targetPower, turnPower);

            Timer.delay(TIME_DELTA);

            prevTime = time;
            prevError = currentError;

            if(Robot.dataCollection.logDataID == DataCollection.LOG_ID_DRIVE && Robot.dataCollection.logDataValues){
                data = new CatzLog(Robot.currentTime.get(), distanceRemain, distanceRemain / distance, currentError, errorRate, turnPower, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
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