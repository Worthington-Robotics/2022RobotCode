#include "subsystems/drivetrain.h"
#include <frc/DriverStation.h>
#include "subsystems/userinput.h"
#include <frc/Errors.h>
#include "SubsystemManager.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace robot
{

    Drivetrain::Drivetrain()
    {

        frontRMod = std::make_shared<SModule>(DRIVE_FR_DRIVE, DRIVE_FR_ANGLE, DRIVE_FR_ENCOD, "front_right", FR_ABS_OFFSET, PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                              PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        frontRMod->setInvertDrive(false);
        sModules.push_back(frontRMod);

        frontLMod = std::make_shared<SModule>(DRIVE_FL_DRIVE, DRIVE_FL_ANGLE, DRIVE_FL_ENCOD, "front_left", FL_ABS_OFFSET, PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                              PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        frontLMod->setInvertDrive(false);
        sModules.push_back(frontLMod);

        rearRMod = std::make_shared<SModule>(DRIVE_RR_DRIVE, DRIVE_RR_ANGLE, DRIVE_RR_ENCOD, "rear_right", RR_ABS_OFFSET, PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                             PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        rearRMod->setInvertDrive(false);
        sModules.push_back(rearRMod);

        rearLMod = std::make_shared<SModule>(DRIVE_RL_DRIVE, DRIVE_RL_ANGLE, DRIVE_RL_ENCOD, "rear_left", RL_ABS_OFFSET, PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF},
                                             PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF});
        rearLMod->setInvertDrive(false);
        sModules.push_back(rearLMod);

        imu = std::make_shared<PigeonIMU>(IMU_ID);

        APPCDiscriptor params = APPCDiscriptor{FIXED_LOOKAHEAD, 0, MAX_ACCEL, 0, PATH_COMPLETE_TOLERANCE};

        PPC = std::make_shared<PurePursuitController>(params);

        reset();

        headingController.setContinuous(true);
        headingController.setInputRange(360);
        headingController.setIMax(5);
    }

    void Drivetrain::createRosBindings(rclcpp::Node *node)
    {
        for (std::shared_ptr<SModule> mod : sModules)
        {
            mod->createRosBindings(node);
        }
        // Create sensor data publishers
        headingController.createRosBindings(node);
        PPC->createRosBindings(node);
        allianceColorPub = node->create_publisher<std_msgs::msg::Int16>("/sys/a_color", rclcpp::SystemDefaultsQoS());


        robotVelocityPub = node->create_publisher<geometry_msgs::msg::Twist>("/drive/dt/velocity", rclcpp::SystemDefaultsQoS());
        robotPositionPub = node->create_publisher<geometry_msgs::msg::Pose2D>("/drive/dt/pose", rclcpp::SystemDefaultsQoS());
        drivetrainHeadingPub = node->create_publisher<std_msgs::msg::Float32>("/drive/dt/heading", rclcpp::SystemDefaultsQoS());
        autoLookaheadPointPub = node->create_publisher<rospathmsgs::msg::Waypoint>("/drive/auto/lookahead_point", rclcpp::SystemDefaultsQoS());
        autoToLookaheadAnglePub = node->create_publisher<std_msgs::msg::Float32>("/drive/auto/lookahead_point_angle", rclcpp::SystemDefaultsQoS());
        driveControlModePub = node->create_publisher<std_msgs::msg::Int16>("/drive/control_mode_echo", rclcpp::SystemDefaultsQoS());
       


#ifdef SystemIndependent
        intakeDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/intake_motor/demand", rclcpp::SystemDefaultsQoS());
        intakeSoleDemandPublisher = node->create_publisher<std_msgs::msg::Int16>("/externIO/intake_solenoid/state", rclcpp::SystemDefaultsQoS());
        indexerDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/indexer_motor/demand", rclcpp::SystemDefaultsQoS());
        deliveryDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/delivery_motor/demand", rclcpp::SystemDefaultsQoS());
        flywheelDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/flywheel_motor/demand", rclcpp::SystemDefaultsQoS());
        hoodDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/hood_motor/demand", rclcpp::SystemDefaultsQoS());
        climberLDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/climber_l_motor/demand", rclcpp::SystemDefaultsQoS());
        climberRDemandPublisher = node->create_publisher<can_msgs::msg::MotorMsg>("/externIO/climber_r_motor/demand", rclcpp::SystemDefaultsQoS());
#endif

        // Create subscribers
        stickSub0 = node->create_subscription<sensor_msgs::msg::Joy>("/sticks/stick0", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::setStick0, this, _1));

        stickSub1 = node->create_subscription<sensor_msgs::msg::Joy>("/sticks/stick1", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::setStick1, this, _1));


        DriveModeSub = node->create_subscription<std_msgs::msg::Int16>("/drive/control_mode", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::setDriveMode, this, _1));

        HeadingControlSub = node->create_subscription<std_msgs::msg::Int16>("/actions/heading_control", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::setHeadingControlEnabled, this, _1));
        limelightAngleOffsetSub = node->create_subscription<std_msgs::msg::Float32>("/limelight/angle_offset", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::setLimelightAngleOffset, this, _1));
        limelightRangeSub = node->create_subscription<std_msgs::msg::Float32>("/limelight/range", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::setLimelightRange, this, _1));
        aiAngleOffsetSub = node->create_subscription<std_msgs::msg::Float32>("/ai/angle_offset", rclcpp::SensorDataQoS(), std::bind(&Drivetrain::setAIAngleOffset, this, _1));

#ifdef SystemIndependent
        hoodResetSub = node->create_subscription<std_msgs::msg::Bool>("/externIO/hood_motor/is_reset", rclcpp::SystemDefaultsQoS(), std::bind(&Drivetrain::setHoodReset, this, _1));
#endif

        // creating clients and services

        startPath = node->create_service<autobt_msgs::srv::StringService>("/drive/start_path", std::bind(&Drivetrain::enablePathFollowerS, this, _1, _2));

        GPClient = node->create_client<rospathmsgs::srv::GetPath>("/get_path");
    }

    void Drivetrain::reset()
    {
        imu->SetFusedHeading(0);

        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR, 10, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_3_GeneralAccel, 251, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_2_GeneralCompass, 249, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_1_General, 10, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel, 253, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 251, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_10_SixDeg_Quat, 249, 0);

        driveState = OPEN_LOOP_ROBOT_REL;

        frontRMod->reset();
        frontLMod->reset();
        rearRMod->reset();
        rearLMod->reset();
    }

    void Drivetrain::onStart()
    {}

    void Drivetrain::updateSensorData()
    {
        //TODO this might be a spot for the PIDF issue
        drivetrainHeadingMsg.data = -(std::fmod((imu->GetFusedHeading() + 360), 360));
        sOdom.Update(frc::Rotation2d{units::degree_t{drivetrainHeadingMsg.data}}, frontRMod->getState(),
                     frontLMod->getState(), rearRMod->getState(), rearLMod->getState());

        pose = sOdom.GetPose();
        robotPositionMsg.x = pose.X().to<double>();
        robotPositionMsg.y = pose.Y().to<double>();
        robotPositionMsg.theta = pose.Rotation().Degrees().to<double>();

        currState = sKinematics.ToChassisSpeeds(frontRMod->getState(), frontLMod->getState(), rearRMod->getState(), rearLMod->getState());
        robotVelocityMsg.linear.x = currState.vx();
        robotVelocityMsg.linear.y = currState.vy();
        robotVelocityMsg.angular.z = currState.omega();

        std::vector<double> joyData = UserInput::scalarCut(lastStick0, DRIVE_STICK_DEADBAND,
                                                           DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);
        stickTwist.linear.x = joyData.at(X_AXIS);
        stickTwist.linear.y = joyData.at(Y_AXIS);
        stickTwist.angular.z = joyData.at(Z_AXIS);

        driveControlModeMsg.data = driveState;
    }

    void Drivetrain::onLoop(double currentTime)
    {
        if (pathQueued != "")
        {
            GPReq = std::make_shared<rospathmsgs::srv::GetPath::Request>();
            GPReq->path_name = pathQueued;
            if (GPClient->service_is_ready())
            {
                std::cout << "Making async request" << std::endl;
                hasPathStart = false;
                future = GPClient->async_send_request(GPReq);
            }
            else
            {
                frc::ReportError(frc::err::UnsupportedInSimulation, "drivetrain.cpp", 400, "enablePathFollower", "You have somehow made a request to the path generation server that it was unable to process, please try again later ;-;");
            }
            pathQueued = "";
        }
        // check if the future is valid from the path service
        if (!hasPathStart && future.valid())
        {
            std::vector<rospathmsgs::msg::Waypoint> path = future.get()->path;
            std::cout << "the total number of points in the ARRAY is: " << path.size() << std::endl;
            std::stack<rospathmsgs::msg::Waypoint> pathStack;
            PPC->mLastpoint = path.back();
            while (path.size() > 1)
            {
                pathStack.push(path.back());
                path.pop_back();
            }
            PPC->setPath(pathStack);

            hasPathStart = true;
            driveState = PURSUIT;
        }

        execActions();

        frc::ChassisSpeeds speed;
        
        if(frc::DriverStation::IsTeleop()){
            driveState = OPEN_LOOP_FIELD_REL;
        }
        switch (driveState)
        {
        case OPEN_LOOP_FIELD_REL:
        {
            // if we are safe, set motor demands,
            if (lastStickTime + DRIVE_TIMEOUT > frc::Timer::GetFPGATimestamp().to<double>())
            {

                // convert to demands
                speed = twistDrive(stickTwist, frc::Rotation2d{units::degree_t{drivetrainHeadingMsg.data}});
            }
            else
            { // otherwise force motors to zero, there is stale data
                std::cout << "drive input stale: " << std::endl;
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
                std::cout << "drive input stale: " << std::endl;
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
            if (!PPC->isDone(sOdom.GetPose()) && !frc::DriverStation::IsTeleop())
            {
                auto [mSpeed, mLookAheadPoint, inertialHeading] = PPC->update(sOdom.GetPose(), currState, currentTime);
                speed = mSpeed;
                lookAheadPoint = mLookAheadPoint;
            }
            else
            {
                std::cout << "Completed path" << std::endl;
                driveState = OPEN_LOOP_FIELD_REL;
            }
            break;
        default:
            frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state, fuck you");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }

        if (headingControl > 0)
        {
            // TODO:: THIS MIGHT BE AN AREA WITH ISSUE REGARDING THE CONTROLLER
            speed.omega = -units::radians_per_second_t{headingController.update(drivetrainHeadingMsg.data)};
        }
        else if (driveState == PURSUIT)
        {
            headingController.setSetpoint(lookAheadPoint.heading, false);
            speed.omega = -units::radians_per_second_t{headingController.update(drivetrainHeadingMsg.data)};
        }

        //speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};

        moduleStates = sKinematics.ToSwerveModuleStates(speed);

        switch (driveState)
        {
        case OPEN_LOOP_FIELD_REL:
        case OPEN_LOOP_ROBOT_REL:
            //FR, FL, RR, RL
            for (int i = 0; i < sModules.size(); i++)
            {
                sModules[i]->setMotors(moduleStates[i]);
            }
            break;
        case PURSUIT: // for now have pursuit as an illegal mode
        case VELOCITY_TWIST:
            // FR, FL, RR, RL
            // std::cout << "requested speed is: " << moduleStates[0].speed.to<double>() << std::endl;

            for (int i = 0; i < sModules.size(); i++)
            {
                sModules[i]->setMotorVelocity(moduleStates[i]);
            }
            break;

        default:
            frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state, fuck you");
            speed = frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s};
        }
        // checkDeltaCurrent(moduleData.frontLeft.angleCurrent, moduleData.frontRight.angleCurrent, moduleData.rearLeft.angleCurrent, moduleData.rearRight.angleCurrent);
        // checkDeltaCurrent(moduleData.frontLeft.driveCurrent, moduleData.frontRight.driveCurrent, moduleData.rearLeft.driveCurrent, moduleData.rearRight.driveCurrent);
    }

    void Drivetrain::publishData()
    {
        for (std::shared_ptr<SModule> mod : sModules)
        {
            mod->publishModuleInfo();
        }
        std_msgs::msg::Int16 allianceColorMsg;
        if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
            allianceColorMsg.data = 1;
        } else {
            allianceColorMsg.data = 2;
        }
        allianceColorPub->publish(allianceColorMsg);
        autoLookaheadPointPub->publish(lookAheadPoint);
        robotPositionPub->publish(robotPositionMsg);
        robotVelocityPub->publish(robotVelocityMsg);
        driveControlModePub->publish(driveControlModeMsg);
        drivetrainHeadingPub->publish(drivetrainHeadingMsg);

#ifdef SystemIndependent
        intakeDemandPublisher->publish(intakeDemandMsg);
        intakeSoleDemandPublisher->publish(intakeSoleDemandMsg);
        indexerDemandPublisher->publish(indexerDemandMsg);
        deliveryDemandPublisher->publish(deliveryDemandMsg);
        //fsngflywheelDemandPublisher->publish(flywheelDemandMsg);
        climberLDemandPublisher->publish(climberDemandMsg);
        climberRDemandPublisher->publish(climberDemandMsg);
        if (hoodReset)
        {
            hoodDemandPublisher->publish(hoodDemandMsg);
        }
#endif
        frc::SmartDashboard::PutNumber("drive/heading", std::fmod(drivetrainHeadingMsg.data + 360, 360));
    }

    // Setters

    void Drivetrain::setLimelightAngleOffset(const std_msgs::msg::Float32 setpoint)
    {
        targetAngleOffset = setpoint.data;
    }

    void Drivetrain::setLimelightRange(const std_msgs::msg::Float32 lRange)
    {
        range = lRange.data;
        #ifdef noRosDebug
        frc::SmartDashboard::PutNumber("limelight/range", range);
        #endif
    }

    void Drivetrain::setAIAngleOffset(const std_msgs::msg::Float32 setpoint)
    {
        ballAngleOffset = setpoint.data;
    }

#ifdef SystemIndependent
    void Drivetrain::setHoodReset(const std_msgs::msg::Bool msg)
    {
        hoodReset = msg.data;
    }
#endif

    void Drivetrain::setDriveMode(const std_msgs::msg::Int16 msg)
    {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

    void Drivetrain::setHeadingControlEnabled(const std_msgs::msg::Int16 engaged)
    {
        headingControl = engaged.data;
        if (headingControl == 1)
        {
            headingControlSetpoint = drivetrainHeadingMsg.data;
        }
        else if (headingControl == 2)
        {
            if(range != -1){
                headingControlSetpoint = drivetrainHeadingMsg.data - targetAngleOffset;
            } else {
                headingControlSetpoint = drivetrainHeadingMsg.data;
            }
        }
        else if (headingControl == 3)
        {
            headingControlSetpoint = drivetrainHeadingMsg.data + ballAngleOffset;
        }
        headingController.setSetpoint(headingControlSetpoint, true);
    }

    void Drivetrain::setStick0(const sensor_msgs::msg::Joy msg)
    {
        lastStickTime = frc::Timer::GetFPGATimestamp().to<double>();
        lastStick0 = msg;
        isRobotRel = lastStick0.buttons.at(0);
        gyroReset = lastStick0.buttons.at(4);
        tankLockButton = lastStick0.buttons.at(5);
    }

    void Drivetrain::setStick1(const sensor_msgs::msg::Joy msg)
    {
        lastStickTime = frc::Timer::GetFPGATimestamp().to<double>();
        lastStick1 = msg;

        if (lastStick1.buttons.at(1)){
            headingControl = 2;
            headingControlUpdate = true;
        } else if (!lastStick1.buttons.at(1) && headingControlUpdate) {
            headingControl = 0;
            headingControlUpdate = false;
        }

        #ifdef systemIndependent
        shoot = lastStick1.buttons.at(0);
        intake = lastStick1.buttons.at(2);
        flywheelButton = lastStick1.buttons.at(4);
        intakeDeploy = lastStick1.buttons.at(6);
        intakeRetract = lastStick1.buttons.at(7);
        unintake = lastStick1.buttons.at(5);
        climberEnabled = lastStick1.buttons.at(9);
        // remaps -1 to 1 axis to 0 to 1
        hoodDemand = ((-lastStick1.axes.at(3) + 1) / 2);
        climberDemand = ((-lastStick1.axes.at(1) * .75));
        #endif
    }

    // Services

    void Drivetrain::enablePathFollowerS(std::shared_ptr<autobt_msgs::srv::StringService_Request> ping, std::shared_ptr<autobt_msgs::srv::StringService_Response> pong)
    {
        enablePathFollower(ping->request_string);
        pong->success = true;
    }

    bool Drivetrain::enablePathFollower(std::string name)
    {
        pathQueued = name;
        return true;
    }

    // Utility Functions

    void Drivetrain::enableOpenLoop()
    {
        driveState = OPEN_LOOP_FIELD_REL;
    }

    void Drivetrain::resetPose()
    {
        sOdom.ResetPosition(frc::Pose2d{frc::Translation2d{0_m, 0_m}, frc::Rotation2d{0_deg}}, frc::Rotation2d{0_deg});
        imu->SetFusedHeading(0);
    }

    void Drivetrain::enableDebug(bool debugEnable)
    {
        DEBUG = debugEnable;
    }

    void Drivetrain::execActions()
    {
        // if (isRobotRel && (driveState == OPEN_LOOP_FIELD_REL || driveState == OPEN_LOOP_ROBOT_REL))
        // {
        //     driveState = OPEN_LOOP_ROBOT_REL;
        // }
        // else if (driveState == OPEN_LOOP_FIELD_REL || driveState == OPEN_LOOP_ROBOT_REL)
        // {
        //     driveState = OPEN_LOOP_FIELD_REL;
        // }

        if (gyroReset)
        {
            imu->SetFusedHeading(0);
        }
        frc::SmartDashboard::PutNumber("drive/heading control", headingControl);
        frc::SmartDashboard::PutNumber("drive/heading control setpoint", std::fmod((headingControlSetpoint + 360), 360));
        if (headingControl == 2 && range != -1)
        {
            headingControlSetpoint = drivetrainHeadingMsg.data - targetAngleOffset;
            headingController.setSetpoint(headingControlSetpoint, false);
        } 
        else if (headingControl == 3)
        {
            headingControlSetpoint = drivetrainHeadingMsg.data + ballAngleOffset;
            headingController.setSetpoint(headingControlSetpoint, false);
        }
        // factor this out into a stuct/class thing!
        // setup for toggle button that puts robot into tankdrive mode!

        // // first, if the button state changes to true change the states to HELD
        // if (tankLockButton && !tankLockHeld)
        // {
        //     tankLockState = !tankLockState;
        //     tankLockHeld = true;
        // }
        // else if (!tankLockButton)
        // {
        //     tankLockHeld = false;
        // }
        // // if in tank drive, overwrite the drive state and set strafing to zero
        // if (tankLockState)
        // {
        //     driveState = OPEN_LOOP_ROBOT_REL;
        //     stickTwist.linear.y = 0;
        // }

#ifdef SystemIndependent
        if (hoodReset)
        {
            hoodDemandMsg.demand = hoodDemand * 800;
            hoodDemandMsg.control_mode = 1;
            hoodDemandMsg.arb_feedforward = 0;
        }

        if (intake)
        {
            intakeDemandMsg.control_mode = 0;
            intakeDemandMsg.demand = .3;
            intakeDemandMsg.arb_feedforward = 0;
            indexerDemandMsg.control_mode = 0;
            indexerDemandMsg.demand = .5;
            indexerDemandMsg.arb_feedforward = 0;
        }
        else if (unintake)
        {
            intakeDemandMsg.control_mode = 0;
            intakeDemandMsg.demand = -.5;
            intakeDemandMsg.arb_feedforward = 0;
            indexerDemandMsg.control_mode = 0;
            indexerDemandMsg.demand = -.5;
            indexerDemandMsg.arb_feedforward = 0;
        }
        else
        {
            intakeDemandMsg.control_mode = 0;
            intakeDemandMsg.demand = 0;
            intakeDemandMsg.arb_feedforward = 0;
            indexerDemandMsg.control_mode = 0;
            indexerDemandMsg.demand = 0;
            indexerDemandMsg.arb_feedforward = 0;
            deliveryDemandMsg.control_mode = 0;
            deliveryDemandMsg.demand = 0;
            deliveryDemandMsg.arb_feedforward = 0;
        }

        if(intakeDeploy){
            intakeSoleDemandMsg.data = 1;
        }
        else if(intakeRetract){
            intakeSoleDemandMsg.data = -1;
        }

        if (shoot)
        {
            deliveryDemandMsg.control_mode = 0;
            deliveryDemandMsg.demand = 1;
            deliveryDemandMsg.arb_feedforward = 0;
        }

        if (flywheelButton && !flywheelHeld)
        {
            flywheelState = !flywheelState;
            flywheelHeld = true;
        }
        else if (!flywheelButton)
        {
            flywheelHeld = false;
        }
        // if in tank drive, overwrite the drive state and set strafing to zero
        if (flywheelState)
        {
            flywheelDemandMsg.control_mode = 2;
            flywheelDemandMsg.demand = 325;
            flywheelDemandMsg.arb_feedforward = 0;
        }
        else
        {
            flywheelDemandMsg.control_mode = 0;
            flywheelDemandMsg.demand = 0;
            flywheelDemandMsg.arb_feedforward = 0;
        }
        if (climberEnabled)
        {
            climberDemandMsg.control_mode = 0;
            climberDemandMsg.demand = climberDemand;
            climberDemandMsg.arb_feedforward = 0;
        }
        else
        {
            climberDemandMsg.control_mode = 0;
            climberDemandMsg.demand = 0;
            climberDemandMsg.arb_feedforward = 0;
        }
#endif
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

    void Drivetrain::setHeadingControlSetpoint(double newHeadingSetpoint)
    {
        headingControlSetpoint = newHeadingSetpoint;
    }

    void Drivetrain::setHeadingControlGains(PIDFDiscriptor nGains)
    {
        headingController.setPIDFDisc(nGains);
    }

} // namespace robot
