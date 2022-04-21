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

namespace robot {

    Drivetrain::Drivetrain() {
#define DISCRIPTORS PIDFDiscriptor{DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF}, PIDFDiscriptor{ANGLE_KP, ANGLE_KI, ANGLE_KD, ANGLE_KF}
        frontRMod = std::make_shared<SModule>(DRIVE_FR_DRIVE, DRIVE_FR_ANGLE, DRIVE_FR_ENCOD, "front_right", FR_ABS_OFFSET, DISCRIPTORS);
        frontLMod = std::make_shared<SModule>(DRIVE_FL_DRIVE, DRIVE_FL_ANGLE, DRIVE_FL_ENCOD, "front_left", FL_ABS_OFFSET, DISCRIPTORS);
        rearRMod = std::make_shared<SModule>(DRIVE_RR_DRIVE, DRIVE_RR_ANGLE, DRIVE_RR_ENCOD, "rear_right", RR_ABS_OFFSET, DISCRIPTORS);
        rearLMod = std::make_shared<SModule>(DRIVE_RL_DRIVE, DRIVE_RL_ANGLE, DRIVE_RL_ENCOD, "rear_left", RL_ABS_OFFSET, DISCRIPTORS);
#undef DISCRIPTORS
        sModules = {frontRMod, frontLMod, rearRMod, rearLMod};
        for (std::shared_ptr<SModule> module : sModules) {
            module->setInvertDrive(false);
        }

        imu = std::make_shared<PigeonIMU>(IMU_ID);
        APPCDiscriptor params = APPCDiscriptor{FIXED_LOOKAHEAD, VELOCITY_LOOKAHEAD_COEFF, 0, MAX_ACCEL, 0, PATH_COMPLETE_TOLERANCE, 0, PURSUIT_X_PID_GAINS, PURSUIT_Y_PID_GAINS};
        PPC = std::make_shared<PurePursuitController>(params);

        reset();

        headingController.setContinuous(true);
        headingController.setInputRange(360);
        headingController.setIMax(5);
    }

    void Drivetrain::createRosBindings(rclcpp::Node *node) {
        for (std::shared_ptr<SModule> module : sModules) {
            module->createRosBindings(node);
        }
        /* Create sensor data publishers */
        headingController.createRosBindings(node);
        PPC->createRosBindings(node);
        allianceColorPub = node->create_publisher<MSG_INT>("/sys/a_color", DEFAULT_QOS);

        robotVelocityPub = node->create_publisher<geometry_msgs::msg::Twist>("/drive/dt/velocity", DEFAULT_QOS);
        robotPositionPub = node->create_publisher<geometry_msgs::msg::Pose2D>("/drive/dt/pose", DEFAULT_QOS);
        drivetrainHeadingPub = node->create_publisher<MSG_FLOAT>("/drive/dt/heading", DEFAULT_QOS);
        autoLookaheadPointPub = node->create_publisher<rospathmsgs::msg::Waypoint>("/drive/auto/lookahead_point", DEFAULT_QOS);
        autoToLookaheadAnglePub = node->create_publisher<MSG_FLOAT>("/drive/auto/lookahead_point_angle", DEFAULT_QOS);
        driveControlModePub = node->create_publisher<MSG_INT>("/drive/control_mode_echo", DEFAULT_QOS);


        /* Create subscribers */
        stickSub0 = node->create_subscription<MSG_JOY>("/sticks/stick0", SENSOR_QOS, std::bind(&Drivetrain::setStick0, this, _1));

        stickSub1 = node->create_subscription<MSG_JOY>("/sticks/stick1", SENSOR_QOS, std::bind(&Drivetrain::setStick1, this, _1));


        DriveModeSub = node->create_subscription<MSG_INT>("/drive/control_mode", DEFAULT_QOS, std::bind(&Drivetrain::setDriveMode, this, _1));

        HeadingControlSub = node->create_subscription<MSG_INT>("/actions/heading_control", DEFAULT_QOS, std::bind(&Drivetrain::setHeadingControlEnabled, this, _1));
        limelightAngleOffsetSub = node->create_subscription<MSG_FLOAT>("/limelight/angle_offset", SENSOR_QOS, std::bind(&Drivetrain::setLimelightAngleOffset, this, _1));
        limelightRangeSub = node->create_subscription<MSG_FLOAT>("/limelight/range", SENSOR_QOS, std::bind(&Drivetrain::setLimelightRange, this, _1));
        aiAngleOffsetSub = node->create_subscription<MSG_FLOAT>("/ai/angle_offset", SENSOR_QOS, std::bind(&Drivetrain::setAIAngleOffset, this, _1));

        /* Creating clients and services */

        startPath = node->create_service<MSG_STRING>("/drive/start_path", std::bind(&Drivetrain::enablePathFollowerS, this, _1, _2));

        GPClient = node->create_client<rospathmsgs::srv::GetPath>("/get_path");
    }

    void Drivetrain::reset() {
        imu->SetFusedHeading(0);

        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR, 10, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_3_GeneralAccel, 251, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_2_GeneralCompass, 249, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_1_General, 10, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel, 253, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 251, 0);
        imu->SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_10_SixDeg_Quat, 249, 0);

        driveState = kOPEN_LOOP_ROBOT_REL;

        for (std::shared_ptr<SModule> module : sModules) {
            module->reset();
        }

        resetPose();
    }

    void Drivetrain::onStart() {
        if (frc::DriverStation::IsTeleop()) {
            driveState = kOPEN_LOOP_FIELD_REL;
        }
    }

    void Drivetrain::updateSensorData() {
        //TODO this might be a spot for the PIDF issue
        drivetrainHeadingMsg.data = -(std::fmod((imu->GetFusedHeading() + 360), 360));
        sOdom.Update(
            frc::Rotation2d{units::degree_t{drivetrainHeadingMsg.data}},
            frontRMod->getState(),
            frontLMod->getState(),
            rearRMod->getState(),
            rearLMod->getState()
        );

        pose = sOdom.GetPose();
        robotPositionMsg.x = pose.X().to<double>();
        robotPositionMsg.y = pose.Y().to<double>();
        robotPositionMsg.theta = pose.Rotation().Degrees().to<double>();

        currState = sKinematics.ToChassisSpeeds(
            frontRMod->getState(),
            frontLMod->getState(),
            rearRMod->getState(),
            rearLMod->getState()
        );
        robotVelocityMsg.linear.x = currState.vx();
        robotVelocityMsg.linear.y = currState.vy();
        robotVelocityMsg.angular.z = currState.omega();

        std::vector<double> joyData = UserInput::scalarCut(lastStick0, DRIVE_STICK_DEADBAND, DRIVE_STICK_POWER, DRIVE_STICK_SCALAR);
        stickTwist.linear.x = joyData.at(X_AXIS);
        stickTwist.linear.y = joyData.at(Y_AXIS);
        stickTwist.angular.z = joyData.at(Z_AXIS);

        driveControlModeMsg.data = driveState;
    }

    void Drivetrain::onLoop(double currentTime) {
        if (pathQueued != "") {
            GPReq = std::make_shared<rospathmsgs::srv::GetPath::Request>();
            GPReq->path_name = pathQueued;
            if (GPClient->service_is_ready()) {
                std::cout << "Making async request" << std::endl;
                hasPathStart = false;
                future = GPClient->async_send_request(GPReq);
            } else {
                frc::ReportError(frc::err::UnsupportedInSimulation, "drivetrain.cpp", 400, "enablePathFollower", "You have somehow made a request to the path generation server that it was unable to process, please try again later ;-;");
            }
            pathQueued = "";
        }
        /* Check if the future is valid from the path service */
        if (!hasPathStart && future.valid()) {
            std::vector<rospathmsgs::msg::Waypoint> path = future.get()->path;
            std::cout << "the total number of points in the ARRAY is: " << path.size() << std::endl;
            std::stack<rospathmsgs::msg::Waypoint> pathStack;
            PPC->mLastpoint = path.back();
            while (path.size() > 1) {
                pathStack.push(path.back());
                path.pop_back();
            }
            PPC->setPath(pathStack);

            hasPathStart = true;
            driveState = kPURSUIT;
        }

        execActions();

        frc::ChassisSpeeds speed;

        switch (driveState) {
            case kOPEN_LOOP_FIELD_REL:
                /* If we are safe, set motor demands */
                if (lastStickTime + DRIVE_TIMEOUT > GET_TIME_DOUBLE) {

                    /* Convert to demands */
                    speed = twistDrive(stickTwist, frc::Rotation2d{units::degree_t{drivetrainHeadingMsg.data}});
                } else { /* Otherwise force motors to zero, there is stale data */
                    std::cout << "drive input stale: " << std::endl;
                    speed = SPEED_STOPPED;
                }
                break;
            case kOPEN_LOOP_ROBOT_REL:
                /* If we are safe, set motor demands */
                if (lastStickTime + DRIVE_TIMEOUT > GET_TIME_DOUBLE) {
                    /* Convert to demands */
                    speed = twistDrive(stickTwist);
                } else { /* Otherwise force motors to zero, there is stale data */
                    std::cout << "drive input stale: " << std::endl;
                    speed = SPEED_STOPPED;
                }
                break;
            case kVELOCITY_TWIST:
                /* If we are safe, set motor demands */
                speed = frc::ChassisSpeeds{2_mps, 0_mps, 0_rad_per_s};

                /* FR, FL, RR, RL */

                break;
            case kPURSUIT:
                if (!PPC->isDone(sOdom.GetPose()) /*&& !frc::DriverStation::IsTeleop()*/) {
                    auto [mSpeed, mLookAheadPoint, inertialHeading] = PPC->update(sOdom.GetPose(), currState, currentTime);
                    speed = mSpeed;
                    lookAheadPoint = mLookAheadPoint;
                }
                else {
                    std::cout << "Completed path" << std::endl;
                    driveState = kOPEN_LOOP_FIELD_REL;
                }
                break;
            default:
                frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state");
                speed = SPEED_STOPPED;
        }
        if (headingControl > 0) {
            // TODO:: THIS MIGHT BE AN AREA WITH ISSUE REGARDING THE CONTROLLER
            speed.omega = -units::radians_per_second_t{headingController.update(drivetrainHeadingMsg.data)};
        } else if (driveState == kPURSUIT) {
            headingController.setSetpoint(lookAheadPoint.heading, false);
            speed.omega = -units::radians_per_second_t{headingController.update(drivetrainHeadingMsg.data)};
        }
        //speed = SPEED_STOPPED;

        moduleStates = sKinematics.ToSwerveModuleStates(speed);

        switch (driveState) {
            case kOPEN_LOOP_FIELD_REL:
            case kOPEN_LOOP_ROBOT_REL:
                /* FR, FL, RR, RL */
                for (int i = 0; i < sModules.size(); i++) {
                    sModules[i]->setMotors(moduleStates[i]);
                }
                break;
            case kPURSUIT: 
            case kVELOCITY_TWIST:
                /* FR, FL, RR, RL */
                // std::cout << "requested speed is: " << moduleStates[0].speed.to<double>() << std::endl;

                for (int i = 0; i < sModules.size(); i++) {
                    sModules[i]->setMotorVelocity(moduleStates[i]);
                }
                break;
            default:
                frc::ReportError(frc::err::InvalidParameter, "drivetrain.cpp", 208, "onLoop()", "Invalid drive state");
                speed = SPEED_STOPPED;
        }
        // checkDeltaCurrent(moduleData.frontLeft.angleCurrent, moduleData.frontRight.angleCurrent, moduleData.rearLeft.angleCurrent, moduleData.rearRight.angleCurrent);
        // checkDeltaCurrent(moduleData.frontLeft.driveCurrent, moduleData.frontRight.driveCurrent, moduleData.rearLeft.driveCurrent, moduleData.rearRight.driveCurrent);
    }

    void Drivetrain::publishData() {
        for (std::shared_ptr<SModule> mod : sModules) {
            mod->publishModuleInfo();
        }
        MSG_INT allianceColorMsg;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
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

        frc::SmartDashboard::PutNumber("drive/heading", std::fmod(drivetrainHeadingMsg.data + 360, 360));
    }

    /* Setters */

    void Drivetrain::setLimelightAngleOffset(const MSG_FLOAT setpoint) {
        targetAngleOffset = setpoint.data;
    }

    void Drivetrain::setLimelightRange(const MSG_FLOAT lRange) {
        range = lRange.data;
#ifdef noRosDebug
        frc::SmartDashboard::PutNumber("limelight/range", range);
#endif
    }

    void Drivetrain::setAIAngleOffset(const MSG_FLOAT setpoint) {
        ballAngleOffset = setpoint.data;
    }

    void Drivetrain::setDriveMode(const MSG_INT msg) {
        std::cout << "changing drivetrain to mode " << msg.data << std::endl;
        driveState = static_cast<ControlState>(msg.data);
    }

    void Drivetrain::setHeadingControlEnabled(const MSG_INT engaged) {
        headingControl = engaged.data;
        switch (headingControl) {
            case 1:
                headingControlSetpoint = drivetrainHeadingMsg.data;
                break;
            case 2:
                if (range != -1) {
                    headingControlSetpoint = drivetrainHeadingMsg.data - targetAngleOffset;
                } else {
                    headingControlSetpoint = drivetrainHeadingMsg.data;
                }
                break;
            case 3:
                headingControlSetpoint = drivetrainHeadingMsg.data + ballAngleOffset;
                break;
        }
        headingController.setSetpoint(headingControlSetpoint, true);
    }

    void Drivetrain::setStick0(const MSG_JOY msg) {
        lastStickTime = GET_TIME_DOUBLE;
        lastStick0 = msg;
        isRobotRel = lastStick0.buttons.at(0);
        gyroReset = lastStick0.buttons.at(4);
        tankLockButton = lastStick0.buttons.at(5);
    }

    void Drivetrain::setStick1(const MSG_JOY msg) {
        lastStickTime = GET_TIME_DOUBLE;
        lastStick1 = msg;

        if (lastStick1.buttons.at(1)) {
            headingControl = 2;
            headingControlUpdate = true;
        } else if (!lastStick1.buttons.at(1) && headingControlUpdate) {
            headingControl = 0;
            headingControlUpdate = false;
        }
    }

    /* Services */

    void Drivetrain::enablePathFollowerS(std::shared_ptr<MSG_STRING_Request> ping, std::shared_ptr<MSG_STRING_Response> pong) {
        enablePathFollower(ping->request_string);
        pong->success = true;
    }

    bool Drivetrain::enablePathFollower(std::string name) {
        pathQueued = name;
        return true;
    }

    /* Utility Functions */

    void Drivetrain::enableOpenLoop() {
        driveState = kOPEN_LOOP_FIELD_REL;
    }

    void Drivetrain::resetPose() {
        sOdom.ResetPosition(
            frc::Pose2d{
                frc::Translation2d{0_m, 0_m},
                frc::Rotation2d{0_deg}
            },
            frc::Rotation2d{0_deg}
        );
        imu->SetFusedHeading(0);
    }

    void Drivetrain::enableDebug(bool debugEnable) {
        DEBUG = debugEnable;
    }

    void Drivetrain::execActions() {
        // if (isRobotRel && (driveState == kOPEN_LOOP_FIELD_REL || driveState == kOPEN_LOOP_ROBOT_REL))
        // {
        //     driveState = kOPEN_LOOP_ROBOT_REL;
        // }
        // else if (driveState == kOPEN_LOOP_FIELD_REL || driveState == kOPEN_LOOP_ROBOT_REL)
        // {
        //     driveState = kOPEN_LOOP_FIELD_REL;
        // }

        if (gyroReset) {
            imu->SetFusedHeading(0);
        }
        frc::SmartDashboard::PutNumber("drive/heading control", headingControl);
        frc::SmartDashboard::PutNumber("drive/heading control setpoint", std::fmod((headingControlSetpoint + 360), 360));
        if (headingControl == 2 && range != -1) {
            headingControlSetpoint = drivetrainHeadingMsg.data - targetAngleOffset;
        }  else if (headingControl == 3) {
            headingControlSetpoint = drivetrainHeadingMsg.data + ballAngleOffset;   
        }
        headingController.setSetpoint(headingControlSetpoint, false);
        /* Factor this out into a struct/class thing! */
        /* Setup for toggle button that puts robot into tankdrive mode! */

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
        //     driveState = kOPEN_LOOP_ROBOT_REL;
        //     stickTwist.linear.y = 0;
        // }
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist, const frc::Rotation2d &orientation) {
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            units::meters_per_second_t{twist.linear.x},
            units::meters_per_second_t{twist.linear.y},
            units::radians_per_second_t{twist.angular.z},
            orientation
        );
        return speeds;
    }

    frc::ChassisSpeeds Drivetrain::twistDrive(const geometry_msgs::msg::Twist &twist) {
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds{
            units::meters_per_second_t{twist.linear.x},
            units::meters_per_second_t{twist.linear.y},
            units::radians_per_second_t{twist.angular.z}
        };
        return speeds;
    }

    void Drivetrain::setHeadingControlSetpoint(double newHeadingSetpoint) {
        headingControlSetpoint = newHeadingSetpoint;
    }

    void Drivetrain::setHeadingControlGains(PIDFDiscriptor nGains) {
        headingController.setPIDFDisc(nGains);
    }

} // namespace robot
