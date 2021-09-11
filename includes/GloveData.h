//
// Created by orange on 2/9/21.
//

#ifndef TEST_GLOVE_SENSEGLOVE_H
#define TEST_GLOVE_SENSEGLOVE_H


#include <SenseGlove.h>

class GloveData {
private:
    SGCore::SG::SenseGlove *m_rightGlove = nullptr;
    SGCore::SG::SenseGlove *m_leftGlove = nullptr;

    SGCore::HandProfile *m_leftProfile = nullptr;
    SGCore::HandProfile *m_rightProfile = nullptr;

    SGCore::HandProfile calibration(SGCore::SG::SenseGlove *glove);
    SGCore::SG::SG_GlovePose getPose(SGCore::SG::SenseGlove *glove);
    SGCore::HandPose getHandPose(SGCore::SG::SenseGlove *glove, SGCore::SG::SG_GlovePose *glovePose, SGCore::HandProfile *profile);
    std::vector<SGCore::Kinematics::Vect3D> getTipPositions(SGCore::SG::SenseGlove *glove, SGCore::SG::SG_GlovePose *glovePose, SGCore::HandProfile *profile);
    void printGloveInfo(SGCore::SG::SenseGlove *glove);

public:
    GloveData(SGCore::HandProfile *profileLeft, SGCore::HandProfile *profileRight);
    GloveData() : GloveData(nullptr, nullptr) {};

    void calibrateLeft();
    void calibrateRight();

    bool hasRightGlove();
    bool hasLeftGlove();

    bool isCalibratedRight();
    bool isCalibratedLeft();

    SGCore::HandPose getRightHandPose();
    SGCore::HandPose getLeftHandPose();
};


#endif //TEST_GLOVE_SENSEGLOVE_H
