//
// Created by orange on 2/9/21.
//

#ifndef TEST_GLOVE_SENSEGLOVE_H
#define TEST_GLOVE_SENSEGLOVE_H


#include <SenseGlove.h>

class GloveData {
private:
    SGCore::SG::SenseGlove m_rightGlove;
    SGCore::SG::SenseGlove m_leftGlove;

    bool hasRight = false, hasLeft = false;
    bool calibRight = false, calibLeft = false;

    SGCore::HandProfile m_leftProfile;
    SGCore::HandProfile m_rightProfile;

    SGCore::SG::SG_GlovePose pose;

    SGCore::HandProfile calibration(SGCore::SG::SenseGlove *glove);
    SGCore::SG::SG_GlovePose getPose(SGCore::SG::SenseGlove *glove);
    void getHandPose(SGCore::SG::SenseGlove *glove, SGCore::HandProfile *profile, SGCore::HandPose &handpose);
    std::vector<SGCore::Kinematics::Vect3D> getTipPositions(SGCore::SG::SenseGlove *glove, SGCore::SG::SG_GlovePose *glovePose, SGCore::HandProfile *profile);
    void printGloveInfo(SGCore::SG::SenseGlove *glove);
    void saveProfile(std::string file, SGCore::HandProfile *profile);
    bool loadProfile(std::string file, SGCore::HandProfile *profile);

public:
    GloveData(SGCore::HandProfile *profileLeft, SGCore::HandProfile *profileRight);
    GloveData() : GloveData(nullptr, nullptr) {};

    void calibrateLeft();
    void calibrateRight();

    void saveRightProfile(std::string file);
    void saveLeftProfile(std::string file);

    bool loadRightProfile(std::string file);
    bool loadLeftProfile(std::string file);

    bool hasRightGlove();
    bool hasLeftGlove();

    bool isCalibratedRight();
    bool isCalibratedLeft();

    void getRightHandPose(SGCore::HandPose &handpose);
    void getLeftHandPose(SGCore::HandPose &handpose);

    std::vector<SGCore::Kinematics::Vect3D> getRightHandTipPositions();
};


#endif //TEST_GLOVE_SENSEGLOVE_H
