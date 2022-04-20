#include "robot_lib/VersionData.h"
#include <frc/Errors.h>
#include <frc/Filesystem.h>
#include <iostream>
#include <wpi/SmallVector.h>

namespace robot {
    void ShowVersionData() {
        int versionNum = 0;
        std::string fileName = frc::filesystem::GetDeployDirectory() + "/version.dat";

        std::cout << "Opening version file: " << fileName << std::endl;

        FILE* version = std::fopen(fileName.c_str(), "r");
        std::fscanf(version, "VERSION_ID=%i", &versionNum);
        std::fclose(version);
        frc::ReportError(frc::warn::Warning, "SubsystemManager.cpp", 19, "ShowVersionData()", std::string("Robot Code Initialized with version ") + std::to_string(versionNum));
    }   
} // namespace robot
