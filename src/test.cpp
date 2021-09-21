//
// Created by orange on 15/9/21.
//

#include <iostream>
#include "GloveData.h"

int main() {
    GloveData glove;
    glove.calibrateRight();
    while(true) {
        std::cout << std::endl;
        std::vector<SGCore::Kinematics::Vect3D> tipPositions = glove.getRightHandTipPositions();
        for (int f = 0; f < tipPositions.size(); f++)
            std::cout << std::to_string(((SGCore::Finger) f)) << ": " << tipPositions[f].ToString()
                      << std::endl; //writes "thumb: ", "index: " etc.
        SGCore::HandPose pose;
        glove.getRightHandPose(pose);
        std::cout << pose.ToString() << " " << std::endl;
        for (int i = 0; i < pose.handAngles.size(); i++) {
            std::cout << i << ": ";
            for (int j = 0; j < pose.handAngles[i].size(); j++)
                std::cout << pose.handAngles[i][j].x*360.0f/(2.0f*3.1415f) << ", "
                << pose.handAngles[i][j].y*360.0f/(2.0f*3.1415f) << ", "
                << pose.handAngles[i][j].z*360.0f/(2.0f*3.1415f)<< " | ";
            std::cout << std::endl;
        }
        getchar();
    }
}
