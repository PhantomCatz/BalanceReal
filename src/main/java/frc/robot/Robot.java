package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.Balance;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.Drive;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {
  
  public static CatzDrivetrain drivetrain;
  public static Balance balance;
  public static Drive drive;

  public static AHRS navX;
  
  public static DataCollection dataCollection;
  public static Timer currentTime;
  public ArrayList<CatzLog> dataArrayList;

  private XboxController xboxDrv;
  private double steerAngle = 0.0;
  private double drivePower = 0.0;
  private double turnPower = 0.0;

  private final double OFFSET_DELAY = 0.5;

  @Override
  public void robotInit()
  {
    xboxDrv = new XboxController(0);
    
    balance = new Balance();
    drive = new Drive();
    drivetrain = new CatzDrivetrain();
    dataCollection = new DataCollection();

    dataArrayList = new ArrayList<CatzLog>();
    dataCollection.dataCollectionInit(dataArrayList);

    currentTime = new Timer();

    navX = new AHRS(Port.kMXP, (byte) 200);
    navX.reset();
    navX.setAngleAdjustment(-navX.getYaw());

    drivetrain.setBrakeMode();
  }

  @Override
  public void robotPeriodic()
  {
    drivetrain.updateShuffleboard();
    SmartDashboard.putNumber("Joystick", steerAngle);
  }
 
  @Override
  public void autonomousInit() {
    dataCollection.updateLogDataID();
    currentTime.reset();
    currentTime.start();
    dataCollection.startDataCollection();

    drivetrain.initializeOffsets();
    Timer.delay(OFFSET_DELAY);

    balance.BalanceInit();
    //Path6();
    drive.DriveStraight(50, 0.1, 0.35, 0.0, 8.0);
    drive.StopDriving();
    balance.StartBalancing();
  }

  public void Path6() { //See slide 6 (2637 Charged Up autonomous Paths in Google Drive) 
    drive.DriveStraight(-40, 0.05, -0.25, 0.0, 8.0);
    drive.DriveStraight(224, 0.05, 0.35, 0.0, 8.0);
    drive.DriveStraight(-224, 0.05, -0.35, 0.0, 8.0);
    drive.DriveStraight(60, 0.05, 0.25, -90.0, 8.0);
    drive.DriveStraight(65, 0.05, 0.25, 0.0, 8.0);
    drive.StopDriving();
    balance.StartBalancing();
  }

  @Override
  public void autonomousPeriodic() {
    if(dataCollection.getLogDataID() == DataCollection.LOG_ID_BALANCE && dataCollection.logDataValues){
      balance.dataCollection();
    }
  }

  @Override
  public void teleopInit() 
  {
    dataCollection.updateLogDataID();
    currentTime.reset();
    currentTime.start();
    dataCollection.startDataCollection();

    drive.StopDriving();
    balance.StopBalancing();
  }

  @Override
  public void teleopPeriodic()
  {
    steerAngle = calcJoystickAngle(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    drivePower = calcJoystickPower(xboxDrv.getLeftX(), xboxDrv.getLeftY());
    turnPower = xboxDrv.getRightX();

    if(drivePower >= 0.1)
    {
      if(Math.abs(turnPower) >= 0.1)
      {
        drivetrain.translateTurn(steerAngle, drivePower, turnPower);
      }
      else
      {
        drivetrain.drive(steerAngle, drivePower);
      }
      if(dataCollection.getLogDataID() == DataCollection.LOG_ID_SWERVE_MODULE && dataCollection.logDataValues){
        drivetrain.dataCollection();
      }
    }
    else if(Math.abs(turnPower) >= 0.1)
    {
      drivetrain.rotateInPlace(turnPower);
      if(dataCollection.getLogDataID() == DataCollection.LOG_ID_SWERVE_MODULE && dataCollection.logDataValues){
        drivetrain.dataCollection();
      }
    }
    else
    {
      drivetrain.setSteerPower(0.0);
      drivetrain.setDrivePower(0.0);
    }

    if(xboxDrv.getStartButtonPressed())
    {
      drivetrain.zeroGyro();
    }
  }

  @Override
  public void disabledInit()
  {
    currentTime.stop();
    drivetrain.setCoastMode();
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public double calcJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
                //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
                angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
                angle = -180 + angle;
            }
        }
        return angle;
    }

    public double calcJoystickPower(double xJoy, double yJoy)
    {
      return (Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }
}
