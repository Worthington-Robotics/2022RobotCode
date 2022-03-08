#include "AutoSelect.h"
#include <frc/Errors.h>

AutoSelect::AutoSelect(std::vector<std::string> autos){
    this->autos = autos;
}

void AutoSelect::createRosBindings(rclcpp::Node * node){
    treeRunner = node->create_client<autobt_msgs::srv::RunTree>("/auto/exec");
}

void AutoSelect::selectAuto(std::string name){
    std::string pathName = name + ".xml";

    if(!treeRunner->service_is_ready()){
        frc::ReportError(frc::err::UnsupportedInSimulation, "AutoSelect.cpp", 16, "selectAuto",
         "You have somehow made a request to the path generation server that it was unable to process, please try again later ;-;");
    } else {
        std::cout << "Running tree " << pathName<< std::endl;
        auto req = std::make_shared<autobt_msgs::srv::RunTree::Request>();
        req->tree_name = pathName;
        auto future = treeRunner->async_send_request(req);
        // while(!future.valid()){

        // }
        // auto res = future.get();
        // if(!res->success){
        //     frc::ReportError(frc::err::TaskError, "AutoSelect.cpp", 27, "selectAuto",
        //  "Failed the start requested tree for reason: " + res->message);
        // }
    }
}