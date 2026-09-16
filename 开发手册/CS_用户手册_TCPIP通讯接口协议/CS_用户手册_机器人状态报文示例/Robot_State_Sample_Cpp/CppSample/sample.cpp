#include "RobotInterface.hpp"

#include <stdio.h>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    std::unique_ptr<RobotInterface> robot = std::make_unique<RobotInterface>();
    if (argc < 2) {
        std::cout << "Need provide Elite Robot IP address" << std::endl;
        return -1;
    }
    if (!robot->loadConfigure("CS_UserManual_Robot_State_Message.txt")) {
        std::cout << "Load Configure file fail. Check file path" << std::endl;
        return -1;
    }
    robot->setExceptionCallback([](const RobotException& exception){
        // do somethings when an exception occurs.
        std::cout << "time stamp: " << exception.timestamp << " ";
        if (exception.exception_type == RobotException::ExceptionType::RUN_TIME_EXCEPTION) {
            std::cout << "Runtime exception" << std::endl;
        } else if (exception.exception_type == RobotException::ExceptionType::ERROR_CODE) {
            std::cout << "Robot error exception" << std::endl;
        }
    });
    if (!robot->connect(argv[1], 30001)) {
        std::cout << "Connect fail check network or IP" << std::endl;
        return -1;
    }
    // Make a robot runtime exception
    robot->sendScript("def func():\n\tabcd(123)\nend\n");
    std::vector<double> joint;
    while (true) {
        if (!robot->isConnect()) {
            return 0;
        }
        joint = robot->getJointActualPos();
        
        for (auto item : joint) {
            printf("%.3lf\t", item);
        }
        printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    return 0;
}