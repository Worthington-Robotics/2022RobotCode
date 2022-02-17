#include "subsystems/drivetrain.h"
#include "Constants.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/userinput.h"
#include <frc/Errors.h>
#include "SubsystemManager.h"
#include "Robot.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

using std::placeholders::_1;

namespace robot
{

    Drivetrain::Drivetrain()
    {

        frontRMod = std::make_shared<SModule>(DRIVE_FR_DRIVE, DRIVE_FR_ANGLE, DRIVE_FR_ENCOD, FR_ABS_OFFSET, PIDF{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                              PIDF{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        frontRMod->setInvertDrive(true);

        frontLMod = std::make_shared<SModule>(DRIVE_FL_DRIVE, DRIVE_FL_ANGLE, DRIVE_FL_ENCOD, FL_ABS_OFFSET, PIDF{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                              PIDF{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        frontLMod->setInvertDrive(true);

        rearRMod = std::make_shared<SModule>(DRIVE_RR_DRIVE, DRIVE_RR_ANGLE, DRIVE_RR_ENCOD, RR_ABS_OFFSET, PIDF{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                             PIDF{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        rearRMod->setInvertDrive(false);

        rearLMod = std::make_shared<SModule>(DRIVE_RL_DRIVE, DRIVE_RL_ANGLE, DRIVE_RL_ENCOD, RL_ABS_OFFSET, PIDF{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                             PIDF{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        rearLMod->setInvertDrive(true);

        imu = std::make_shared<PigeonIMU>(IMU_ID);


        APPCDiscriptor params = APPCDiscriptor{.25, 0, .5, 0, .05};
        PPC = std::make_shared<PurePursuitController>(params);

        reset();
    }

    void Drivetrain::createRosBindings(rclcpp::Node *node)
    {
        // Create sensor data publishers
        imuPub = node->create_publisher<sensor_msgs::msg::Imu>("/drive/imu", rclcpp::SensorDataQoS());
        robotVelPub = node->create_publisher<geometry_msgs::msg::Twist>("/drive/vel", rclcpp::SystemDefaultsQoS());
        robotPosPub = node->create_publisher<geometry_msgs::msg::Pose2D>("/drive/pose", rclcpp::SystemDefaultsQoS());
        goalPub = node->create_publisher<std_msgs::msg::Float32>("/drive/motor_goal",  rclcpp::SystemDefaultsQoS());
        autoTwistDemandPub = node->create_publisher<geometry_msgs::msg::Twist>("/drive/auto_twist_demand",  rclcpp::SystemDefaultsQoS());
        lookaheadPointPub = node->create_publisher<rospathmsgs::msg::Waypoint>("/drive/lookahead_point",  rclcpp::SystemDefaultsQoS());
        currentAnglePub = node->create_publisher<std_msgs::msg::Float32>("/drive/current_angle",  rclcpp::SystemDefaultsQoS());
        desiredAnglePub = node->create_publisher<std_msgs::msg::Float32>("/drive/desired_angle",  rclcpp::SystemDefaultsQoS());


        // Create subscribers
        trajectorySub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/drive/active_traj", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::trajectoryCallback, this, _1));
        twistSub = node->create_subscription<geometry_msgs::msg::Twist>("/drive/velocity_twist", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::twistCallback, this, _1));
        stickSub = node->create_subscription<sensor_msgs::msg::Joy>(DRIVE_STICK_TOPIC, rclcpp::SensorDataQoS(), std::bind(&Drivetrain::stickCallback, this, _1));

        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/drive/drive_mode", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::driveModeCallback, this, _1));
    
        GPClient = node->create_client<rospathmsgs::srv::GetPath>("/get_path");

    }

    void Drivetrain::reset()
    {
        
        // reset cached data to prevent nullptrs

        // TODO reset sensors

        // Reset the IMU message and wait for data
        imuMsg = sensor_msgs::msg::Imu();
        //set covariances
        imuMsg.orientation_covariance = IMU_ORIENT_COVAR;
        imuMsg.linear_acceleration_covariance = IMU_ACCEL_COVAR;
        imuMsg.angular_velocity_covariance = IMU_ANG_VEL_COVAR;

        // default values to zero
        imu->SetFusedHeading(0);

        imuMsg.linear_acceleration.x = 0;
        imuMsg.linear_acceleration.y = 0;
        imuMsg.linear_acceleration.z = 0;
        imuMsg.orientation.x = 0;
        imuMsg.orientation.y = 0;
        imuMsg.orientation.z = 0;
        imuMsg.orientation.w = 1;
        imuMsg.angular_velocity.x = 0;
        imuMsg.angular_velocity.y = 0;
        imuMsg.angular_velocity.z = 0;

        // clear demands and reset time
        leftDemand = rightDemand = 0;
        lastTwistTime = 0;

        driveState = OPEN_LOOP_ROBOT_REL;

        frontRMod->reset();
        frontLMod->reset();
        rearRMod->reset();
        rearLMod->reset();
    }

    void Drivetrain::onStart()
    {
    }



    void Drivetrain::updateSensorData()
    {
        /* stuff for live changing PID values
        resetPIDVals = frc::SmartDashboard::GetBoolean("Drive/setPID", false);
        if(resetPIDVals){
            kF = frc::SmartDashboard::GetNumber("Drive/PIDkF", 0);
            kP = frc::SmartDashboard::GetNumber("Drive/PIDkP", 0);
            kI = frc::SmartDashboard::GetNumber("Drive/PIDkI", 0);
            kD = frc::SmartDashboard::GetNumber("Drive/PIDkD", 0);
            frontLMod->updateDrivePID({kP, kI, kD, kF});
            frontRMod->updateDrivePID({kP, kI, kD, kF});
            rearLMod->updateDrivePID({kP, kI, kD, kF});
            rearRMod->updateDrivePID({kP, kI, kD, kF});
        }*/
        auto pose = sOdom.GetPose();
        robotPosMsg.x = pose.X().to<double>();
        robotPosMsg.y = pose.Y().to<double>();
        robotPosMsg.theta = pose.Rotation().Degrees().to<double>();
        
        pose = sOdom.GetPose();
        robotPosMsg.x = pose.X().to<double>();
        robotPosMsg.y = pose.Y().to<double>();
        robotPosMsg.theta = pose.Rotation().Degrees().to<double>();

        moduleData = {frontLMod->getData(), frontRMod->getData(), rearLMod->getData(), rearRMod->getData()};

        //frc::DriverStation::ReportWarning("Updating drive sensor data");
        // read the current IMU state
        int16_t accelData[3];
        imu->GetBiasedAccelerometer(accelData);
        // Convert from 2^14 = 1g = 9.8 m/s^2
        imuMsg.linear_acceleration.x = accelData[0] * .000598784;
        imuMsg.linear_acceleration.y = accelData[1] * .000598784;
        imuMsg.linear_acceleration.z = accelData[2] * .000598784;

        double gyroData[3];
        imu->GetRawGyro(gyroData);
        // Convert from deg/s to rad/s
        imuMsg.angular_velocity.x = gyroData[0] * 0.01745329;
        imuMsg.angular_velocity.y = gyroData[1] * 0.01745329;
        imuMsg.angular_velocity.z = gyroData[2] * 0.01745329;

        double orientData[4];
        imu->Get6dQuaternion(orientData);
        imuMsg.orientation.w = orientData[0];
        imuMsg.orientation.x = orientData[1];
        imuMsg.orientation.y = orientData[2];
        imuMsg.orientation.z = orientData[3];

        yaw.data = -imu->GetFusedHeading();
        sOdom.Update(frc::Rotation2d{units::degree_t{yaw.data}}, frontRMod->getState(),
            frontLMod->getState(), rearRMod->getState(), rearLMod->getState());
        if(isRobotRel && (driveState == OPEN_LOOP_FIELD_REL || driveState == OPEN_LOOP_ROBOT_REL))
        {
            driveState = OPEN_LOOP_ROBOT_REL;
        } else if (driveState == OPEN_LOOP_FIELD_REL || driveState == OPEN_LOOP_ROBOT_REL){
            driveState = OPEN_LOOP_FIELD_REL;
        }
    }

    // Average the wheel state velocities TODO fix!!
    double getFwdVelocity(sensor_msgs::msg::JointState wheelState)
    {
        return (wheelState.velocity.at(0) + wheelState.velocity.at(0)) / 2.0;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist, const frc::Rotation2d &orientation)
    {
        double xSpeed = twist.linear.x;
        double ySpeed = twist.linear.y;
        double zTurn = twist.angular.z;
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t{xSpeed}, 
            units::meters_per_second_t{ySpeed}, 
            units::radians_per_second_t{zTurn}, 
            orientation);
        return speeds;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist)
    {
        double xSpeed = twist.linear.x;
        double ySpeed = twist.linear.y;
        double zTurn = twist.angular.z;
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds{
            units::meters_per_second_t{xSpeed},
            units::meters_per_second_t{ySpeed},
            units::radians_per_second_t{zTurn}};
        return speeds;
    }

    void Drivetrain::execActions(){
        if(gyroReset){
            imu->SetFusedHeading(0);
        }
        //Apply a spin lock based on button 2 (set when the raw joystick message is recieved) 
        if(spinLock) {
            //set spin to zero, pretty straight forward (maybe add z axis dumb pid?)
            stickTwist.angular.z = 0;
        }

        //factor this out into a stuct/class thing!
        //setup for toggle button that puts robot into tankdrive mode!

        //first, if the button state changes to true change the states to HELD
        if(tankLockButton && !tankLockHeld) {
            tankLockState = !tankLockState;
            tankLockHeld = true;
        } else if(!tankLockButton){
            tankLockHeld = false;
        }
        //if in tank drive, overwrite the drive state and set stafing to zero
        if(tankLockState){
            driveState = OPEN_LOOP_ROBOT_REL;
            stickTwist.linear.y = 0;
        }
    }

    void Drivetrain::onLoop(double currentTime)
    {
        frc::ChassisSpeeds speed;
        // parse the joy message
        std::vector<double> joyData = UserInput::scalarCut(lastStick, DRIVE_STICK_DEADBAND,
                                                            DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);
        stickTwist.linear.x = joyData.at(1);
        stickTwist.linear.y = joyData.at(0);
        stickTwist.angular.z = joyData.at(4);

       

        execActions();
        auto currState = sKinematics.ToChassisSpeeds(frontRMod->getState(), frontLMod->getState(), rearRMod->getState(), rearLMod->getState());
        robotVelMsg.linear.x = currState.vx();
        robotVelMsg.linear.y = currState.vy();
        robotVelMsg.angular.z = currState.omega();
        goal.data = moduleData.frontLeft.driveGoal;
        switch (driveState)
        {
        case OPEN_LOOP_FIELD_REL:
        {
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp().to<double>())
            {

                // convert to demands
                speed = twistDrive(stickTwist, frc::Rotation2d{units::degree_t{yaw.data}});
            }
            else
            { // otherwise force motors to zero, there is stale data
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }
            break;
        }
        case OPEN_LOOP_ROBOT_REL:
        {
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp().to<double>())
            {
                // convert to demands
                speed = twistDrive(stickTwist);
            }
            else
            { // otherwise force motors to zero, there is stale data
                speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
            }
            break;
        }
        case VELOCITY_TWIST:
            // if we are safe, set motor demands,
            speed = frc::ChassisSpeeds{2_mps, 0_mps, 0_rad_per_s};

            // FR, FL, RR, RL

            break;
        case PURSUIT:
            if(!PPC->isDone(sOdom.GetPose())){
                auto [mSpeed, mLookAheadPoint] = PPC->update(sOdom.GetPose(), currState, currentTime);
                autoTwistDemand.linear.x = mSpeed.vx.to<double>();
                autoTwistDemand.linear.y = mSpeed.vy.to<double>();
                autoTwistDemand.angular.z = mSpeed.omega.to<double>();
                speed = mSpeed;
                lookAheadPoint = mLookAheadPoint;
            } else {
                driveState = OPEN_LOOP_FIELD_REL;
            }
            break;

        default:
            frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state, fuck you");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }

        moduleStates = sKinematics.ToSwerveModuleStates(speed);
        rotationalData moduleOne;

        switch (driveState)
        {
        case OPEN_LOOP_FIELD_REL:
        case OPEN_LOOP_ROBOT_REL:
            // FR, FL, RR, RL
            moduleOne = frontRMod->setMotors(moduleStates[0]);
            #ifdef DEBUG_enable
                currentAngle.data = frontRMod->getData().angleRel;
                desiredAngle.data = moduleOne.angleTicks;
            #endif
           // moduleOne.speed
            frontLMod->setMotors(moduleStates[1]);
            rearRMod->setMotors(moduleStates[2]);
            rearLMod->setMotors(moduleStates[3]);
            break;
        case PURSUIT: // for now have pursuit as an illegal mode
        case VELOCITY_TWIST:
            // FR, FL, RR, RL
            std::cout << "requested speed is: " << moduleStates[0].speed.to<double>() << std::endl;
            frontRMod->setMotorVelocity(moduleStates[0]);
            frontLMod->setMotorVelocity(moduleStates[1]);
            rearRMod->setMotorVelocity(moduleStates[2]);
            rearLMod->setMotorVelocity(moduleStates[3]);
            break;

        default:
            frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state, fuck you");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }
        checkDeltaCurrent(moduleData.frontLeft.angleCurrent, moduleData.frontRight.angleCurrent, moduleData.rearLeft.angleCurrent, moduleData.rearRight.angleCurrent);
        checkDeltaCurrent(moduleData.frontLeft.driveCurrent, moduleData.frontRight.driveCurrent, moduleData.rearLeft.driveCurrent, moduleData.rearRight.driveCurrent);
        
    }

    void Drivetrain::resetPose()
    {
        sOdom.ResetPosition(frc::Pose2d{frc::Translation2d{0_m, 0_m}, frc::Rotation2d{0_deg}}, frc::Rotation2d{0_deg});
        imu->SetFusedHeading(0);
    }

    void Drivetrain::publishData()
    {
        lookaheadPointPub->publish(lookAheadPoint);
        robotPosPub->publish(robotPosMsg);
        robotVelPub->publish(robotVelMsg);
        autoTwistDemandPub->publish(autoTwistDemand);
        goalPub->publish(goal);
        imuPub->publish(imuMsg);
        #ifdef DEBUG_enable
            currentAnglePub->publish(currentAngle);
            desiredAnglePub->publish(desiredAngle);
        #endif

        if(DEBUG){
            frc::SmartDashboard::PutNumber("Drive/Front/Left/AngleABS", moduleData.frontLeft.encAbs);
            frc::SmartDashboard::PutNumber("Drive/Front/Right/AngleABS", moduleData.frontRight.encAbs);
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/AngleABS", moduleData.rearLeft.encAbs);
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/AngleABS", moduleData.rearRight.encAbs);

            frc::SmartDashboard::PutNumber("Drive/Front/Right/Desired", moduleStates[0].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Front/Left/Desired", moduleStates[1].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/Desired", moduleStates[2].angle.Degrees().to<double>());
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/Desired", moduleStates[3].angle.Degrees().to<double>());

            frc::SmartDashboard::PutNumber("Drive/Front/Left/UncalABS", std::fmod(moduleData.frontLeft.encAbs - FR_ABS_OFFSET + 360, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Front/Right/UncalABS", std::fmod(moduleData.frontRight.encAbs - FL_ABS_OFFSET + 360, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Rear/Left/UncalABS", std::fmod(moduleData.rearLeft.encAbs - RL_ABS_OFFSET + 360, 360.0));
            frc::SmartDashboard::PutNumber("Drive/Rear/Right/UncalABS", std::fmod(moduleData.rearRight.encAbs - RR_ABS_OFFSET + 360, 360.0));
        }

        frc::SmartDashboard::PutNumber("Drive/Front/Left/AngleRel", moduleData.frontLeft.angleRel);
        frc::SmartDashboard::PutNumber("Drive/Front/Right/AngleRel", moduleData.frontRight.angleRel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/AngleRel", moduleData.rearLeft.angleRel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/AngleRel", moduleData.rearRight.angleRel);

        frc::SmartDashboard::PutNumber("Drive/Control_Mode", static_cast<unsigned int>(driveState));

        frc::SmartDashboard::PutString("Drive/Pose/units", sOdom.GetPose().X().name());
        frc::SmartDashboard::PutNumber("Drive/Pose/X", sOdom.GetPose().X().to<double>());
        frc::SmartDashboard::PutNumber("Drive/Pose/Y", sOdom.GetPose().Y().to<double>());
        frc::SmartDashboard::PutNumber("Drive/Pose/Theta", yaw.data);

        frc::SmartDashboard::PutNumber("Drive/Front/Right/Angle/Current", moduleData.frontRight.angleCurrent);
        frc::SmartDashboard::PutNumber("Drive/Front/Left/Angle/Current", moduleData.frontLeft.angleCurrent);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/Angle/Current", moduleData.rearRight.angleCurrent);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/Angle/Current", moduleData.rearLeft.angleCurrent);

        frc::SmartDashboard::PutNumber("Drive/Front/Right/Drive/Current", moduleData.frontRight.driveCurrent);
        frc::SmartDashboard::PutNumber("Drive/Front/Left/Drive/Current", moduleData.frontLeft.driveCurrent);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/Drive/Current", moduleData.rearRight.driveCurrent);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/Drive/Current", moduleData.rearLeft.driveCurrent);

        frc::SmartDashboard::PutNumber("Drive/Front/Right/Drive/Vel", moduleData.frontRight.driveVel);
        frc::SmartDashboard::PutNumber("Drive/Front/Left/Drive/Vel", moduleData.frontLeft.driveVel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/Drive/Vel", moduleData.rearRight.driveVel);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/Drive/Vel", moduleData.rearLeft.driveVel);

        frc::SmartDashboard::PutNumber("Drive/Front/Right/Drive/Goal", moduleData.frontRight.driveGoal);
        frc::SmartDashboard::PutNumber("Drive/Front/Left/Drive/Goal", moduleData.frontLeft.driveGoal);
        frc::SmartDashboard::PutNumber("Drive/Rear/Right/Drive/Goal", moduleData.rearRight.driveGoal);
        frc::SmartDashboard::PutNumber("Drive/Rear/Left/Drive/Goal", moduleData.rearLeft.driveGoal);
    }


    void Drivetrain::enablePathFollower(std::string name)
    {
        GPReq = std::make_shared<rospathmsgs::srv::GetPath::Request>();
        GPReq->path_name = name;
        if(GPClient->service_is_ready()){
            std::cout << "Hey batter batter, hey batter batter";
            auto future = GPClient->async_send_request(GPReq);
            std::cout << "SWING!" << std::endl;
            while(rclcpp::ok() && !future.valid())
            {
                std::cout << "Waiting for path *Jepordy Theme Plays*" << std::endl;
                //rclcpp::spin_some(Robot::getSubManager());
            }
            std::vector<rospathmsgs::msg::Waypoint> path = future.get()->path; 
            std::cout << "the total number of point in the ARRAY is: " << path.size() << std::endl;
            std::stack<rospathmsgs::msg::Waypoint> pathStack;
            PPC->mLastpoint = path.back();
            while(path.size() > 1)
            {
                pathStack.push(path.back());
                path.pop_back();
            }
            PPC->setPath(pathStack);
            driveState = PURSUIT;
        } else {
            frc::ReportError(frc::err::UnsupportedInSimulation, "drivetrain.cpp", 400, "enablePathFollower", "You have somehow made a request to the path generation server that it was unable to process, please try again later ;-;");
        }
        return;
    }

    void Drivetrain::enableOpenLoop(){
        driveState = OPEN_LOOP_FIELD_REL;
    }

    void Drivetrain::trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
    }

    /**
     * Updates the twist lockout timer, and the latest twist information.
     * the units on the twist indicies are mode specific.
     **/
    void Drivetrain::twistCallback(const geometry_msgs::msg::Twist msg)
    {
        lastTwistTime = frc::Timer::GetFPGATimestamp().to<double>();
        lastTwist = msg;
    }

    void Drivetrain::stickCallback(const sensor_msgs::msg::Joy msg)
    {
        lastStickTime = frc::Timer::GetFPGATimestamp().to<double>();
        lastStick = msg;
        //update this in disabled? or just init publish empty data? (null ptr on boot)
        isRobotRel = lastStick.buttons.at(0);
        spinLock = lastStick.buttons.at(1);
        gyroReset = lastStick.buttons.at(4);
        tankLockButton = lastStick.buttons.at(5);
        
    }

    void Drivetrain::driveModeCallback(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

    void Drivetrain::enableDebug(bool debugEnable){
        if(debugEnable)
        frc::ReportError(frc::warn::Warning, "drivetrain.cpp", 344, "debug", "You've fucked up, here's your debug data, Good luck! :D");
        DEBUG = debugEnable;
    }

    void Drivetrain::checkDeltaCurrent(double currentOne, double currentTwo, double currentThree, double currentFour){
        std::vector<double> arr = {currentOne, currentTwo, currentThree, currentFour};

        // frc::SmartDashboard::PutNumber("Drive/Current1", currentOne);
        // frc::SmartDashboard::PutNumber("Drive/Current2", currentTwo);
        // frc::SmartDashboard::PutNumber("Drive/Current3", currentThree);
        // frc::SmartDashboard::PutNumber("Drive/Current4", currentFour);

        for(int i = 0; i < 4; i++){
            double average = 0;
            for(int k = 0; k < 4; k++){
                if(i != k){
                    average += arr.at(k);
                }
            }
            average /= 3.0;

            //frc::SmartDashboard::PutNumber("Drive/CurrentAvg" + i, average);

            if(arr.at(i) > average + DELTA_CURRENT_THRESHOLD){
                iterators.at(i)++;
            } else {
                iterators.at(i)--;
            }

            if(iterators.at(i) > 50 && iterators.at(i) < 55){
                if(i == 0)
                    frc::ReportError(frc::err::ParameterOutOfRange, "drivetrain.cpp", 362, "currentDelta", "Drivetrain current is too high in the front left. Current is " + std::to_string(arr.at(i))
                    + ". The average current of the other three modules is " + std::to_string(average));
                if(i == 1)
                    frc::ReportError(frc::err::ParameterOutOfRange, "drivetrain.cpp", 362, "currentDelta", "Drivetrain current is too high in the front right. Current is " + std::to_string(arr.at(i))
                    + ". The average current of the other three modules is " + std::to_string(average));
                if(i == 2)
                    frc::ReportError(frc::err::ParameterOutOfRange, "drivetrain.cpp", 362, "currentDelta", "Drivetrain current is too high in the rear left. Current is " + std::to_string(arr.at(i))
                    + ". The average current of the other three modules is " + std::to_string(average));
                if(i == 3)
                    frc::ReportError(frc::err::ParameterOutOfRange, "drivetrain.cpp", 362, "currentDelta", "Drivetrain current is too high in the rear right. Current is " + std::to_string(arr.at(i))
                    + ". The average current of the other three modules is " + std::to_string(average));    
            }
        }
    }
   
} // namespace robot
