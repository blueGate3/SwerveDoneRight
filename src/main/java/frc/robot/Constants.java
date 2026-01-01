package frc.robot;

public class Constants {
 
    public class DriveConst {

            //CAN IDs for all our drivesystem
            public static final int FLDrive = 1;
            public static final int FLTURN = 2;
            public static final int FRDrive = 3;
            public static final int FRTurn = 4;
            public static final int BRDrive = 5;
            public static final int BRTurn = 6;
            public static final int BLDrive = 7;
            public static final int BLTurn = 8;
    
            /* 
            * - m/s to rpm formula: RPM = ((Velocity in m/s)/(circumference)) *60 (you multiply by 60 to convert revolutions per second to revolutions per minute)
            * - with gear ratio: rpm of output = rpm of motor * (gear ratio/output gear teeth)
            */
            //TODO update module-specific constants when new modules ordered (gear ratio, max speed, etc.)
            public static final double kWheelDiameter = .1016; // 0.1016 M wheel diameter (4")
            public static final double kWheelCircumference = Math.PI * kWheelDiameter;
            public static final double turningWheelGearRatio = 150/7; //standard steering gear ratio on MK4i 
            public static final double drivingWheelGearRatio = 6.12; //L3 gear ratio for driving
            public static final double rotationsToMetersScaler = (kWheelCircumference/drivingWheelGearRatio);
            public static final double rpmToVelocityScaler = (kWheelCircumference)/(60*drivingWheelGearRatio); //SDS Mk4I standard gear ratio from motor to wheel, divide by 60 to go from secs to mins
            public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
            public static final double kMaxSpeed = 1; // 5.88 meters per second or 19.3 ft/s (max speed of SDS Mk4i with Vortex motor)
            public static final double turnEncoderScaler = 2* Math.PI;
            public static final double kMaxAccel = 8;
            public static final double kP = 0.072;//0.228;
            public static final double kI = 0.0; //.1
            public static final double kD = 0.2; //0.0095;
            public static final double kVelocityTolerance = 0.001; // m/s        
            public static final double kS = 0.05;
            public static final double kV = 0.17499;
            public static final double kA = 3.44;
            public static final double kClosedLoopRampRate = 0.5;
            public static final int kMaxDriveAmps = 50;
    }   
    public class TrajectoryConst {
        public static final double kMaxSpeed = DriveConst.kMaxSpeed;
        public static final double kMaxAcceleration = 1;
    }
}