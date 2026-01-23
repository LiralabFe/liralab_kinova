#include <KinovaLiralab.hpp>

namespace KinovaLiralab
{
    namespace KORTEX = Kinova::Api;
    
    Robot::Robot()
    {
        std::cout << "Connecting to robot..." << std::endl;

        // High level
        _transport = new KORTEX::TransportClientTcp();
        _router = new KORTEX::RouterClient(_transport, KinovaLiralab::Robot::ErrorCallback);
        _session = new KORTEX::SessionManager(_router);
        // Low level (real time)
        _transportRealTime = new KORTEX::TransportClientUdp();
        _routerRealTime = new KORTEX::RouterClient(_transportRealTime, KinovaLiralab::Robot::ErrorCallback);
        _sessionRealTime = new KORTEX::SessionManager(_routerRealTime);

        // connect
        _sessionInfo = KORTEX::Session::CreateSessionInfo();
        _sessionInfo.set_username("admin");
        _sessionInfo.set_password("admin");
        _sessionInfo.set_session_inactivity_timeout(60000);
        _sessionInfo.set_connection_inactivity_timeout(2000);
        _transport->connect("192.168.1.10", PORT);
        _transportRealTime->connect("192.168.1.10",PORT_REALTIME);

        _session->CreateSession(_sessionInfo);
        _sessionRealTime->CreateSession(_sessionInfo);

        _base = new KORTEX::Base::BaseClient(_router);
        _baseRealTime = new KORTEX::BaseCyclic::BaseCyclicClient(_routerRealTime);

        // setup kdl
        _urdfModel.initFile("/home/legion/ROS/kinova_ws/src/ros2_kortex/kortex_description/robots/gen3.urdf");
        if(!kdl_parser::treeFromUrdfModel(_urdfModel, _kdlTree)) Print("[Error while parsing urdf]\n"); 
        else Print("[URDF Loaded]\n");
        _kdlTree.getChain("base_link","end_effector_link",_robotChain);
        _kdlSolver = new KDL::ChainFkSolverPos_recursive(_robotChain);

        KDL::JntArray kdlJoints(_robotChain.getNrOfJoints());
        KORTEX::Base::JointAngles angles = _base->GetMeasuredJointAngles();
        for(int i = 0; i < 7; i++)
            kdlJoints(i) = angles.joint_angles(i).value() * M_PI / 180.0f;

        KDL::Frame eeFrame;
        _kdlSolver->JntToCart(kdlJoints,eeFrame);
        std::cout << eeFrame.p.x() << " , " << eeFrame.p.y() << " , " << eeFrame.p.z() << std::endl;
        // ---
        KORTEX::Base::Pose pose = _base->GetMeasuredCartesianPose();
        std::cout << pose.x() << " , " << pose.y() << " , " << pose.z() << std::endl;
        return;

        KDL::Jacobian jacobian(7);
        KDL::ChainJntToJacSolver jacobian_solver(_robotChain);
        jacobian_solver.JntToJac(kdlJoints,jacobian);
        for (unsigned int i = 0; i < jacobian.rows(); i++)
            for (unsigned int j = 0; j < jacobian.columns(); j++)
                _jacobian(i,j) = jacobian(i,j);
    }

    Robot::~Robot()
    {
        std::cout << "--[DISCONNECTING]--" << std::endl;
        auto actuator_config = KORTEX::ActuatorConfig::ActuatorConfigClient(_router);
        auto control_mode_message = KORTEX::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(KORTEX::ActuatorConfig::ControlMode::POSITION);

        for(int i = 1; i < 8; i++)
            actuator_config.SetControlMode(control_mode_message, i);
        
        std::cout << "Closing..." << std::endl;
        _session->CloseSession();
        _sessionRealTime->CloseSession();
        _router->SetActivationStatus(false);
        _routerRealTime->SetActivationStatus(false);
        _transport->disconnect();
        _transportRealTime->disconnect();
        delete _transport;
        delete _transportRealTime;
        delete _router;
        delete _routerRealTime;
        delete _session;
        delete _sessionRealTime;
        delete _base;
        delete _baseRealTime;
        delete _kdlSolver;
    }

    int64_t Robot::GetTickUs()
    {
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);   
        return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
    }

    void Robot::PrintException(KORTEX::KDetailedException& ex)
    {
        // You can print the error informations and error codes
        auto error_info = ex.getErrorInfo().getError();
        std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
        
        std::cout << "KError error_code: " << error_info.error_code() << std::endl;
        std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
        std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

        // Error codes by themselves are not very verbose if you don't see their corresponding enum value
        // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
        std::cout << "Error code string equivalent: " << KORTEX::ErrorCodes_Name(KORTEX::ErrorCodes(error_info.error_code())) << std::endl;
        std::cout << "Error sub-code string equivalent: " << KORTEX::SubErrorCodes_Name(KORTEX::SubErrorCodes(error_info.error_sub_code())) << std::endl;
    }

    void Robot::GoHome()
    {
        KORTEX::Base::ServoingModeInformation servoingInformation = KORTEX::Base::ServoingModeInformation();
        servoingInformation.set_servoing_mode(KORTEX::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        _base->SetServoingMode(servoingInformation);

        KORTEX::Base::RequestedActionType actionType = KORTEX::Base::RequestedActionType();
        actionType.set_action_type(KORTEX::Base::REACH_JOINT_ANGLES);
        KORTEX::Base::ActionList actionList = _base->ReadAllActions(actionType);
        KORTEX::Base::ActionHandle actionHandle = KORTEX::Base::ActionHandle();
        actionHandle.set_identifier(0);
        
        for(KORTEX::Base::Action a : actionList.action_list())
            if(a.name() == "Home") actionHandle = a.handle();

        if(actionHandle.identifier() == 0) return;

        bool actionFinished = false;
        KORTEX::Common::NotificationOptions option = KORTEX::Common::NotificationOptions();
        KORTEX::Common::NotificationHandle notificationHandle = _base->OnNotificationActionTopic(KinovaLiralab::Robot::CheckForEndOrAbort(actionFinished), option);
        _base->ExecuteActionFromReference(actionHandle);
    
        while(!actionFinished)  std::this_thread::sleep_for(ACTION_WAITING_TIME);
        Print("[Home Reached]\n");
        _base->Unsubscribe(notificationHandle);
    }

    void Robot::EvaluateJacobianKDL()
    {
        KDL::JntArray kdlJoints(_robotChain.getNrOfJoints());
        KORTEX::Base::JointAngles angles = _base->GetMeasuredJointAngles();
        for(int i = 0; i < 7; i++)
            kdlJoints(i) = angles.joint_angles(i).value() * M_PI / 180.0f;

        KDL::Jacobian jacobian(7);
        KDL::ChainJntToJacSolver jacobian_solver(_robotChain);
        jacobian_solver.JntToJac(kdlJoints,jacobian);
        for (unsigned int i = 0; i < jacobian.rows(); i++)
            for (unsigned int j = 0; j < jacobian.columns(); j++)
                _jacobian(i,j) = jacobian(i,j);

        // std::cout << _jacobian << std::endl;
    }

    void Robot::EvaluateJacobianKDLNumerically()
    {
        Eigen::MatrixXd J(6, 7);
        float eps = 1e-4;

        // FK di riferimento
        KDL::Frame T0;
        KDL::ChainFkSolverPos_recursive fk_solver(_robotChain);
        KDL::JntArray q(7);
        KORTEX::Base::JointAngles angles = _base->GetMeasuredJointAngles();
        for(int i = 0; i < 7; i++)
            q(i) = angles.joint_angles(i).value() * M_PI / 180.0f;
        fk_solver.JntToCart(q, T0);

        Eigen::Vector3d p0(T0.p.x(), T0.p.y(), T0.p.z());
        Eigen::Matrix3d R0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                R0(r, c) = T0.M(r, c);

        for (int i = 0; i < 7; ++i)
        {
            KDL::JntArray q_pert = q;
            q_pert(i) += eps;

            KDL::Frame Ti;
            fk_solver.JntToCart(q_pert, Ti);

            Eigen::Vector3d pi(Ti.p.x(), Ti.p.y(), Ti.p.z());
            Eigen::Matrix3d Ri;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    Ri(r, c) = Ti.M(r, c);

            // ------- Traslazione -------
            Eigen::Vector3d dp = (pi - p0) / eps;

            // ------- Rotazione -------
            Eigen::Matrix3d dR = Ri * R0.transpose();
            Eigen::AngleAxisd aa(dR);
            Eigen::Vector3d domega = aa.axis() * aa.angle() / eps;

            // ------- Colonna i -------
            J.block<3,1>(0, i) = dp;
            J.block<3,1>(3, i) = domega;
        }
        _jacobian = J;
        std::cout << _jacobian<< std::endl;
    }

    void Robot::EvaluateJacobian()
    {
        auto servoingMode = KORTEX::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(KORTEX::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        _base->SetServoingMode(servoingMode);
        
        KORTEX::Base::JointAngles jointAngles = _base->GetMeasuredJointAngles();
        KORTEX::Base::JointAngles jointAnglesEps;

        double eps = 1e-4;
        
        Eigen::VectorXd q(7);
        q(0) = jointAngles.joint_angles(0).value();
        q(1) = jointAngles.joint_angles(1).value();
        q(2) = jointAngles.joint_angles(2).value();
        q(3) = jointAngles.joint_angles(3).value();
        q(4) = jointAngles.joint_angles(4).value();
        q(5) = jointAngles.joint_angles(5).value();
        q(6) = jointAngles.joint_angles(6).value();
        
        KORTEX::Base::Pose pose;
        KORTEX::Base::Pose poseEps;
        KORTEX::RouterClientSendOptions sendOption;
        sendOption.timeout_ms = 1000;
        try{pose =_base->ComputeForwardKinematics(jointAngles,0U, sendOption);}
        catch (KORTEX::KDetailedException& ex){Print("Unable to compute forward kinematics.(1)\n"); PrintException(ex); return;}
        
        Eigen::VectorXd T0(6);
        Eigen::VectorXd Teps(6);
        T0 << pose.x(), pose.y(), pose.z(), pose.theta_x(), pose.theta_y(), pose.theta_z();
        
        for(int i = 0; i < 7; ++i)
        {
            jointAnglesEps = jointAngles;
            jointAnglesEps.mutable_joint_angles(i)->set_value(q(i) + eps);
            try{poseEps =_base->ComputeForwardKinematics(jointAnglesEps, 0U, sendOption);}
            catch (const std::exception &e){Print("Unable to compute forward kinematics.(2)\n"); return;}
            
            Teps << poseEps.x(), poseEps.y(), poseEps.z(), poseEps.theta_x(), poseEps.theta_y(), poseEps.theta_z();
            
            _jacobian.col(i) = (Teps - T0)/eps;
        }
        std::cout << _jacobian << std::endl;
    }

    void Robot::VelocityControl()
    {
        bool return_status = true;
        KORTEX::BaseCyclic::Feedback base_feedback;
        KORTEX::BaseCyclic::Command  base_command;

        std::vector<float> commands;
        Eigen::MatrixXd jpi(7,6); // jacobian pseudo-inverse
        Eigen::VectorXd desiredVelocity(6);
        desiredVelocity << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

        auto servoingMode = KORTEX::Base::ServoingModeInformation();

        int timer_count = 0;
        int64_t now = 0;
        int64_t last = 0;
        int64_t lastSlow = 0;

        int timeout = 0;
        std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
        try
        {
            servoingMode.set_servoing_mode(KORTEX::Base::ServoingMode::LOW_LEVEL_SERVOING);
            _base->SetServoingMode(servoingMode);
            base_feedback = _baseRealTime->RefreshFeedback();

            int actuator_count = _base->GetActuatorCount().count();

            for(int i = 0; i < actuator_count; i++)
            {
                commands.push_back(base_feedback.actuators(i).position());
                base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
            }

            auto realtimeCallback = [](const KORTEX::Error &err, const KORTEX::BaseCyclic::Feedback data)
            {
                // realtime callback
                //std::cout << err.DebugString() << std::endl;
            };

            EvaluateJacobianKDL();
            servoingMode.set_servoing_mode(KORTEX::Base::ServoingMode::LOW_LEVEL_SERVOING);
            _base->SetServoingMode(servoingMode);
            jpi = _jacobian.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::MatrixXd jointDesiredVelocity = jpi * desiredVelocity;

            while(timer_count < (10 * 1000))
            {
                now = GetTickUs();
                if(now - lastSlow > 30000)
                {
                    EvaluateJacobianKDL();
                    jpi = _jacobian.completeOrthogonalDecomposition().pseudoInverse();
                    jointDesiredVelocity = jpi * desiredVelocity;
                    lastSlow = GetTickUs();
                }
                //auto start = std::chrono::high_resolution_clock::now();
                if(now - last > 1000)
                {
                    for(int i = 0; i < actuator_count; i++)
                    {
                        commands[i] += (0.001f * jointDesiredVelocity(i));
                        base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                    }

                    try
                    {
                        _baseRealTime->Refresh_callback(base_command, realtimeCallback, 0);
                    }
                    catch(...)
                    {
                        timeout++;
                    }
                    
                    timer_count++;
                    last = GetTickUs();
                    //auto end = std::chrono::high_resolution_clock::now();
                    //std::cout << "Execution time: " << std::chrono::duration<double>(end-start).count() << std:: endl;
                }
            }
        }
        catch (KORTEX::KDetailedException& ex)
        {
            std::cout << "Kortex error: " << ex.what() << std::endl;
            return_status = false;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "Runtime error: " << ex2.what() << std::endl;
            return_status = false;
        }
    
        // Set back the servoing mode to Single Level Servoing
        servoingMode.set_servoing_mode(KORTEX::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        _base->SetServoingMode(servoingMode);
    }

    void Robot::VelocityControlHighLevel()
    {
        KORTEX::Base::JointSpeeds joint_speeds;
        Eigen::MatrixXd jpi(7,6); // jacobian pseudo-inverse
        Eigen::VectorXd desiredVelocity(6);
        desiredVelocity << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        Eigen::MatrixXd jointDesiredVelocity = jpi * desiredVelocity;
        float timer = 0.0f;
        while(timer < 10.0f)
        {
            EvaluateJacobianKDL();
            jpi = _jacobian.completeOrthogonalDecomposition().pseudoInverse();
            jointDesiredVelocity = jpi * desiredVelocity;
            for (size_t i = 0 ; i < 7; ++i)
            {
                auto joint_speed = joint_speeds.add_joint_speeds();
                joint_speed->set_joint_identifier(i);
                joint_speed->set_value(jointDesiredVelocity(i));
            }
            _base->SendJointSpeedsCommand(joint_speeds);
            joint_speeds.Clear();
            timer += 0.021f;
        }
        // Stop the robot
        std::cout << "Stopping the robot" << std::endl;
        _base->Stop();
    }

    void Robot::TorqueControlExample()
    {
        unsigned int actuator_count = _base->GetActuatorCount().count();
        
        KORTEX::BaseCyclic::Feedback base_feedback;
        KORTEX::BaseCyclic::Command  base_command;
        auto actuator_config = KORTEX::ActuatorConfig::ActuatorConfigClient(_router);

        std::vector<float> commands;

        auto servoing_mode = KORTEX::Base::ServoingModeInformation();

        int timer_count = 0;
        int64_t now = 0;
        int64_t last = 0;

        KDL::Vector gravity(0.0,0.0,-9.81);
        KDL::ChainDynParam dynSolver(_robotChain, gravity);
        KDL::JntArray q(7);                         // [RAD]    current position not limited in range [0-2PI]
        KDL::JntArray qPrev(7);                     // [RAD]    prev positions
        KDL::JntArray qVel(7);                      // [RAD/s]  current velocity
        KDL::JntArray eq(7);                        // [RAD]    equilibrium positions
        KDL::JntArray eqVel(7);                     // [RAD/s]  equilibrium velocity
        KDL::JntArray tau(7),g(7);
        Eigen::VectorXd Kp(7), Kd(7);
        Kp << 60, 50, 40, 30, 25, 10, 5;
        Kd << 10,  5,  5,  3,  2,  1,  0.5;
        try
        {
            // Set the base in low-level servoing mode
            servoing_mode.set_servoing_mode(KORTEX::Base::ServoingMode::LOW_LEVEL_SERVOING);
            _base->SetServoingMode(servoing_mode);
            base_feedback = _baseRealTime->RefreshFeedback();

            // Initialize each actuator to their current position
            for (unsigned int i = 0; i < actuator_count; i++)
            {
                commands.push_back(base_feedback.actuators(i).position());

                // Save the current actuator position, to avoid a following error
                base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
                q(i)        = base_feedback.actuators(i).position() * M_PI / 180.0;
                qPrev(i)    = base_feedback.actuators(i).position() * M_PI / 180.0;
                eq(i)       = base_feedback.actuators(i).position() * M_PI / 180.0;
                eqVel(i)    = 0;
            }

            // Send a first frame
            base_feedback = _baseRealTime->Refresh(base_command);
            
            // Set actuators in torque mode now that the command is equal to measure
            auto control_mode_message = KORTEX::ActuatorConfig::ControlModeInformation();
            control_mode_message.set_control_mode(KORTEX::ActuatorConfig::ControlMode::TORQUE);
            
            for(int i = 1; i < 8; i++)  // NOTE!!! Joint Device IDs are from [1-8]
                actuator_config.SetControlMode(control_mode_message, i);

            // Real-time loop
            while (timer_count < (10 * 1000))
            {
                now = GetTickUs();
                if (now - last > 1000)
                {
                    for(int i = 0; i < 7; i++)
                    {
                        // Position command to first actuator is set to measured one to avoid following error to trigger
                        // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                        //        actuator continues to move under torque command, resulting position error with command will
                        //        trigger a following error and switch back the actuator in position command to hold its position
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

                        /* --- Current angle in range [0,2PI] --- */
                        double currQ = base_feedback.actuators(i).position() * M_PI / 180.0;
                        
                        /* --- Unwrap angles --- */
                        double delta = currQ - qPrev(i);
                        if(delta > M_PI) delta -= 2.0 * M_PI;
                        else if (delta < -M_PI) delta += 2.0 * M_PI;

                        /* --- Update angles with unwrapped angles --- */
                        qPrev(i) = currQ;
                        q(i) += delta;
                        qVel(i) = base_feedback.actuators(i).velocity() * M_PI / 180.0;
                    }

                    /* --- Get gravity compensation --- */
                    dynSolver.JntToGravity(q,g);
                    for(int i=0;i<7;i++)
                        tau(i) = g(i) + Kp(i) * (eq(i) - q(i)) + Kd(i) * (eqVel(i) - qVel(i));

                    /* --- Saturate torque --- */
                    double tau_max[7] = {30,30,30,30,20,20,10};
                    for(int i=0;i<7;i++)
                        tau(i) = std::clamp(tau(i), -tau_max[i], tau_max[i]);

                    /* --- Set torque command --- */
                    for(int i = 0; i < 7; i++)
                        base_command.mutable_actuators(i)->set_torque_joint(tau(i) * 1.05);

                    std::cout << timer_count/1000.0 << std::endl;

                    /* --- Increase identifier to reject out-of-date commands --- */
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);

                    for (int idx = 0; idx < actuator_count; idx++)
                        base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

                    /* --- SEND TORQUE COMMAND --- */
                    try
                    {
                        base_feedback = _baseRealTime->Refresh(base_command, 0);
                    }
                    catch (KORTEX::KDetailedException& ex)
                    {
                        std::cout << "Kortex exception: " << ex.what() << std::endl;

                        std::cout << "Error sub-code: " << KORTEX::SubErrorCodes_Name(KORTEX::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                    }
                    catch (std::runtime_error& ex2)
                    {
                        std::cout << "runtime error: " << ex2.what() << std::endl;
                    }
                    catch(...)
                    {
                        std::cout << "Unknown error." << std::endl;
                    }
                    
                    timer_count++;
                    last = GetTickUs();
                }
            }

            std::cout << "Torque control example completed" << std::endl;

            control_mode_message.set_control_mode(KORTEX::ActuatorConfig::ControlMode::POSITION);

            for(int i = 1; i < 8; i++)
                actuator_config.SetControlMode(control_mode_message, i);

            std::cout << "Torque control example clean exit" << std::endl;

        }
        catch (KORTEX::KDetailedException& ex)
        {
            std::cout << "API error: " << ex.what() << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "Error: " << ex2.what() << std::endl;
        }
        
        // Set the servoing mode back to Single Level
        servoing_mode.set_servoing_mode(KORTEX::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        _base->SetServoingMode(servoing_mode);

        // Wait for a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    void Robot::WeightlessMode()
    {
        unsigned int actuator_count = _base->GetActuatorCount().count();
        
        KORTEX::BaseCyclic::Feedback base_feedback;
        KORTEX::BaseCyclic::Command  base_command;
        auto actuator_config = KORTEX::ActuatorConfig::ActuatorConfigClient(_router);

        std::vector<float> commands;

        auto servoing_mode = KORTEX::Base::ServoingModeInformation();

        int timer_count = 0;
        int64_t now = 0;
        int64_t last = 0;

        KDL::Vector gravity(0.0,0.0,-9.81);
        KDL::ChainDynParam dynSolver(_robotChain, gravity);
        KDL::JntArray q(7);                         // [RAD]    current position not limited in range [0-2PI]
        KDL::JntArray qPrev(7);                     // [RAD]    prev positions
        KDL::JntArray qVel(7);                      // [RAD/s]  current velocity
        KDL::JntArray eq(7);                        // [RAD]    equilibrium positions
        KDL::JntArray eqVel(7);                     // [RAD/s]  equilibrium velocity
        KDL::JntArray tau(7),g(7);
        Eigen::VectorXd Kp(7), Kd(7);
        Kp << 60, 50, 40, 30, 25, 10, 5;
        Kd << 10,  5,  5,  3,  2,  1,  0.5;
        try
        {
            // Set the base in low-level servoing mode
            servoing_mode.set_servoing_mode(KORTEX::Base::ServoingMode::LOW_LEVEL_SERVOING);
            _base->SetServoingMode(servoing_mode);
            base_feedback = _baseRealTime->RefreshFeedback();

            // Initialize each actuator to their current position
            for (unsigned int i = 0; i < actuator_count; i++)
            {
                commands.push_back(base_feedback.actuators(i).position());

                // Save the current actuator position, to avoid a following error
                base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
                q(i)        = base_feedback.actuators(i).position() * M_PI / 180.0;
                qPrev(i)    = base_feedback.actuators(i).position() * M_PI / 180.0;
                eq(i)       = base_feedback.actuators(i).position() * M_PI / 180.0;
                eqVel(i)    = 0;
            }

            // Send a first frame
            base_feedback = _baseRealTime->Refresh(base_command);
            
            // Set actuators in torque mode now that the command is equal to measure
            auto control_mode_message = KORTEX::ActuatorConfig::ControlModeInformation();
            control_mode_message.set_control_mode(KORTEX::ActuatorConfig::ControlMode::TORQUE);
            
            for(int i = 1; i < 8; i++)  // NOTE!!! Joint Device IDs are from [1-8]
                actuator_config.SetControlMode(control_mode_message, i);

            // Real-time loop
            while (timer_count < (10 * 1000))
            {
                now = GetTickUs();
                if (now - last > 1000)
                {
                    for(int i = 0; i < 7; i++)
                    {
                        // Position command to first actuator is set to measured one to avoid following error to trigger
                        // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                        //        actuator continues to move under torque command, resulting position error with command will
                        //        trigger a following error and switch back the actuator in position command to hold its position
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

                        /* --- Current angle in range [0,2PI] --- */
                        double currQ = base_feedback.actuators(i).position() * M_PI / 180.0;
                        
                        /* --- Unwrap angles --- */
                        double delta = currQ - qPrev(i);
                        if(delta > M_PI) delta -= 2.0 * M_PI;
                        else if (delta < -M_PI) delta += 2.0 * M_PI;

                        /* --- Update angles with unwrapped angles --- */
                        qPrev(i) = currQ;
                        q(i) += delta;
                        qVel(i) = base_feedback.actuators(i).velocity() * M_PI / 180.0;
                    }

                    /* --- Get gravity compensation --- */
                    dynSolver.JntToGravity(q,g);
                    for(int i=0;i<7;i++)
                        tau(i) = g(i) + Kp(i) * (eq(i) - q(i)) + Kd(i) * (eqVel(i) - qVel(i));

                    /* --- Saturate torque --- */
                    double tau_max[7] = {30,30,30,30,20,20,10};
                    for(int i=0;i<7;i++)
                        tau(i) = std::clamp(tau(i), -tau_max[i], tau_max[i]);

                    /* --- Set torque command --- */
                    for(int i = 0; i < 7; i++)
                        base_command.mutable_actuators(i)->set_torque_joint(tau(i) * 1.05);

                    std::cout << timer_count/1000.0 << std::endl;

                    /* --- Increase identifier to reject out-of-date commands --- */
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);

                    for (int idx = 0; idx < actuator_count; idx++)
                        base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

                    /* --- SEND TORQUE COMMAND --- */
                    try
                    {
                        base_feedback = _baseRealTime->Refresh(base_command, 0);
                    }
                    catch (KORTEX::KDetailedException& ex)
                    {
                        std::cout << "Kortex exception: " << ex.what() << std::endl;

                        std::cout << "Error sub-code: " << KORTEX::SubErrorCodes_Name(KORTEX::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                    }
                    catch (std::runtime_error& ex2)
                    {
                        std::cout << "runtime error: " << ex2.what() << std::endl;
                    }
                    catch(...)
                    {
                        std::cout << "Unknown error." << std::endl;
                    }
                    
                    timer_count++;
                    last = GetTickUs();
                }
            }

            std::cout << "Torque control example completed" << std::endl;

            control_mode_message.set_control_mode(KORTEX::ActuatorConfig::ControlMode::POSITION);

            for(int i = 1; i < 8; i++)
                actuator_config.SetControlMode(control_mode_message, i);

            std::cout << "Torque control example clean exit" << std::endl;

        }
        catch (KORTEX::KDetailedException& ex)
        {
            std::cout << "API error: " << ex.what() << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "Error: " << ex2.what() << std::endl;
        }
        
        // Set the servoing mode back to Single Level
        servoing_mode.set_servoing_mode(KORTEX::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        _base->SetServoingMode(servoing_mode);

        // Wait for a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    }
}

/*
Per il controllo di impedenza serve il LOW LEVEL in torque mode.
C'è l'esempio:
https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_cpp/examples/108-Gen3_torque_control/01-torque_control_cyclic.cpp

Usalo come base di partenza per fare un esempio di "weightless" robot:
torque = g(q)
per calcolare g usa:
KDL::Vector gravity = (0.0, 0.0, -9.81); // <--- occhio al segno, coerente con il frame base
KDL::ChainDynParam dynSolver(kdl_chain, gravity);
KDL::JntArray q(7);
KDL::JntArray g(7);
dynSolver.JntToGravity(q,g);

!! Occhio a q, devono essere in range [-pi, pi], guarda l'ultima chat di chatGPT

*/