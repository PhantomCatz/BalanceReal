package frc.Mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.robot.Robot;

public class Balance
{
    public static Timer timer = new Timer();
    public static double prevPitchAngle;
    public static double yAxis;
    public static double prevTime = -0.1;
    public static double time = 0;

    public Boolean startBalance = false;

    public double pitchAngle = 0.0;
    public double angleRate = 0.0;
    public double power = 0.0;

    public CatzLog data;

    public final double ANG_SLOWBAND = 10; 
    public final double ANG_GAIN = 0.01;
    public final double RATE_GAIN = 0.025;
    public final double DRV_GAIN = 1;
    public final double MAX_POWER = 0.175;
    public final double TIME_DELTA = 0.1;

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

    public Balance() {}

    public void BalanceInit(){
        final Thread balanceThread = new Thread()
        {
            public void run()
            {
                timer.reset();
                timer.start();

                while(true)
                {
                    pitchAngle = Robot.navX.getPitch();
                    
                    if(startBalance)
                    {
                        time = timer.get();
                        angleRate = (pitchAngle - prevPitchAngle)/(time - prevTime);
                                                
                        power = Clamp(-MAX_POWER, pitchAngle * ANG_GAIN + angleRate * RATE_GAIN, MAX_POWER);
                        if(Math.abs(pitchAngle) < ANG_SLOWBAND)
                        {
                            power = power/2;
                        }
                        Robot.drivetrain.autoDrive(power);   

                        SmartDashboard.putNumber("Pitch", pitchAngle);
                        SmartDashboard.putNumber("Pitch Rate", angleRate);
                        SmartDashboard.putNumber("Power", power);

                        prevPitchAngle = pitchAngle;
                        prevTime = time;
                    }
                    Timer.delay(TIME_DELTA);
                }
            }
        };

        balanceThread.start();
    }

    public void StartBalancing()
    {
        startBalance = true;

        timer.reset();
        timer.start();
    }

    public void StopBalancing()
    {
        startBalance = false;
    }

    public void dataCollection()
    {
        data = new CatzLog(Robot.currentTime.get(), pitchAngle, angleRate, power, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);  
        Robot.dataCollection.logData.add(data);
    }
}