#include "robot_lib/VersionData.h"
#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <iostream>
#include <wpi/SmallVector.h>

namespace robot
{
    void ShowVersionData(){
        int versionNum = 0;
        wpi::SmallVector<char, 300> fn;
        frc::filesystem::GetDeployDirectory(fn);
        std::string fileName(fn.begin(), fn.end());
        fileName += "/version.dat";

        std::cout << "Opening version file: " << fileName << std::endl;

        FILE* version = std::fopen(fileName.c_str(), "r");
        std::fscanf(version, "%*s%i", &versionNum);
        std::fclose(version);
        frc::DriverStation::GetInstance().ReportWarning(std::string("Robot Code Initialized with version ") + std::to_string(versionNum));
    }
    
} // namespace robot
