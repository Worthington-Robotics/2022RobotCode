#include "AutoSelect.h"
#include <frc/Errors.h>
using std::placeholders::_1;

AutoSelect::AutoSelect(std::vector<std::string> autos)
{
    this->autos = autos;
}

void AutoSelect::createRosBindings(rclcpp::Node *node)
{
    treeRunner = node->create_client<autobt_msgs::srv::RunTree>("/auto/exec");
    delimAutoListSub = node->create_subscription<std_msgs::msg::String>("/auto/delim_list", 10, bind(&AutoSelect::saveAuto, this, _1));
}

void AutoSelect::saveAuto(const std_msgs::msg::String msg)
{
    if (!isAutoLoaded)
    {
        std::string autoList = msg.data;
        size_t delimE = autoList.find('-');
        size_t delimB = 1;
        while (delimE != std::string::npos)
        {
            std::cout << "the beginning of the next auto name is: " << delimB << " and the end is: " << delimE << std::endl;
            std::string autoName = autoList.substr(delimB, delimE - delimB);
            if(autoName.length() > 4 && autoName.length() < autoList.length()){
                autos.push_back(autoName);
            }
            std::cout << "the next auto name is: " << autoName << std::endl;
            delimB = delimE + 1;
            delimE = autoList.find('-', delimB);
        }
        autos.push_back(autoList.substr(delimB));
        frc::SmartDashboard::PutStringArray("Auto List", autos);
        isAutoLoaded = true;
    }
}

void AutoSelect::selectAuto(std::string name)
{
    std::string pathName = name;

    if (!treeRunner->service_is_ready())
    {
        frc::ReportError(frc::err::UnsupportedInSimulation, "AutoSelect.cpp", 16, "selectAuto",
                         "You have somehow made a request to the auto executor server that it was unable to process, please try again later ;-;");
    }
    else
    {
        std::cout << "Running tree " << pathName << std::endl;
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