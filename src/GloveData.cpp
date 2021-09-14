//
// Created by orange on 2/9/21.
//

#include "GloveData.h"
/*
A console application demonstrating the intended use of a SenseGlove interface.
Used to compile the programs in the SGCoreCpp/test folder.
*/

#include <iostream> //Output to console

#include <thread>  //To pause the main() while vibrating
#include <chrono>  //To pause the thread for std::chrono::seconds

#include "Library.h"	// Access library details
#include "DeviceList.h" // Access devices and communication state
#include "SenseGlove.h" // SenseGlove interface through which we access data.

/// <summary> Replace "static void BasicTest()" with int main() to compile as a console application. </summary>
//static void BasicTest()

GloveData::GloveData(SGCore::HandProfile *profileLeft, SGCore::HandProfile *profileRight) {
    std::vector<SGCore::SG::SenseGlove> glovesConnected;
    if (!SGCore::DeviceList::SenseCommRunning())
        throw std::runtime_error("Sense Comm not running.");
    glovesConnected = SGCore::SG::SenseGlove::GetSenseGloves(true);
    if (glovesConnected.empty())
        throw std::runtime_error("No Sense Gloves Detected!");
    else {
        if (glovesConnected[0].IsRight()) {
            m_rightGlove = &glovesConnected[0];
            if (glovesConnected.size() > 1) m_leftGlove = &glovesConnected[1];
        } else {
            m_leftGlove = &glovesConnected[0];
            if (glovesConnected.size() > 1) m_rightGlove = &glovesConnected[1];
        }
    }
    if (profileLeft != nullptr)
        m_leftProfile = profileLeft;
    if (profileRight != nullptr)
        m_rightProfile = profileRight;

    if (hasRightGlove()) printGloveInfo(m_leftGlove);
    if (hasLeftGlove()) printGloveInfo(m_leftGlove);
}

void GloveData::calibrateLeft() {
    if (hasLeftGlove()) {
        SGCore::HandProfile profile = calibration(m_leftGlove);
        m_leftProfile = &profile;
    } else
        throw std::runtime_error("No Left Glove Connected!");
}

void GloveData::calibrateRight() {
    if (hasRightGlove()) {
        SGCore::HandProfile profile = calibration(m_rightGlove);
        m_rightProfile = &profile;
    } else
        throw std::runtime_error("No Right Glove Connected!");
}

bool GloveData::hasRightGlove() {
    return m_rightGlove != nullptr;
}

bool GloveData::hasLeftGlove() {
    return m_leftGlove != nullptr;
}

SGCore::HandProfile GloveData::calibration(SGCore::SG::SenseGlove *glove) {
    std::cout << "Connected to a " << (glove->IsRight() ? "right" : "left") << "-handed SenseGlove. Staring calibration" << std::endl;

    /*
    Our goal is to find the min / max sensor values, which correspond to the user opening their hand and making a fist.
    We can only update this range after parsing sensor data, which happens when accessing sensorData, glovePoses or handPoses.
    In our VR use cases, we pull new hand data each frame, and so this min/max range is updated automatically.
    In this example, we will update the range twice; once when the hand is 'open', once when it is closed into a fist.
    */

    // Step 1: Open hand - Calibrates extension
    std::cout << std::endl;
    std::cout << "Step 1: Place your hand on a flat surface, like a table, and spread your thumb and fingers." << std::endl;
    // Once you get the hang of this motion, you can do it without the surface.
    std::cout << "Once your hand is in the right position, press any key to continue" << std::endl;
    getchar(); //system("pause");

    // This function updates the calibration range of testGlove.
    glove->UpdateCalibrationRange(); // Instead of this method, you can also use the GetSensorData(), GetGlovePose() or GetHandPose function instead.


    // Step 2: Fist - Calibrates flexion
    std::cout << std::endl;
    std::cout << "Step 2: Close your hand into a fist. Make sure your fingers aren't wrapped around your thumb." << std::endl;
    std::cout << "Once your hand is in the right position, press any key to continue" << std::endl;
    getchar(); //system("pause");

    // This function updates the calibration range of testGlove.
    glove->UpdateCalibrationRange();


    // At this point, we've collected data while the hand was open, and when it was closed.
    // The calibration range should now have the two extremes to interpolate between.
    // Let's check & ouput the ranges:
    std::vector<SGCore::Kinematics::Vect3D> minRanges, maxRanges;
    glove->GetCalibrationRange(minRanges, maxRanges);

    // The calibration ranges contain the x, y, z values, which represent the pronation/supination, flexion/extension, and abduction/adduction movements respectively, in radians.
    // For readability's sake, we'll print out the flexion/extension values in degrees.
    float rad2Deg = 180 / M_PI;
    std::cout << std::endl << "Evaluated the following calibration range for extension/flexion" << std::endl;
    std::cout << "Extensions: ";
    for (int f = 0; f < minRanges.size(); f++)
    {
        std::cout << std::to_string((int)(rad2Deg * minRanges[f].y));
        if (f < minRanges.size() - 1) { std::cout << ", "; }
    }
    std::cout << std::endl << "Flexions: ";
    for (int f = 0; f < maxRanges.size(); f++)
    {
        std::cout << std::to_string((int)(rad2Deg * maxRanges[f].y));
        if (f < maxRanges.size() - 1) { std::cout << ", "; }
    }
    std::cout << std::endl;

    // Now we apply the calibration to a default profile
    SGCore::HandProfile cachedProfile = SGCore::HandProfile::Default(glove->IsRight()); //a default profile for the right-sided glove.
    glove->ApplyCalibration(cachedProfile);

    return cachedProfile;
}

bool GloveData::isCalibratedLeft() {
    return m_leftProfile != nullptr;
}

bool GloveData::isCalibratedRight() {
    return m_rightProfile != nullptr;
}

SGCore::SG::SG_GlovePose GloveData::getPose(SGCore::SG::SenseGlove *glove) {
    //Retrieving Glove Pose: The position / rotation of the glove, as well as its sensor angles placed in the right direction.
    SGCore::SG::SG_GlovePose glovePose;
    if (glove->GetGlovePose(glovePose))
        return glovePose;
    else
        throw std::runtime_error("Failed to get glove pose.");
}

SGCore::HandPose GloveData::getHandPose(SGCore::SG::SenseGlove *glove, SGCore::SG::SG_GlovePose *glovePose, SGCore::HandProfile *profile) {
    //As an example, lets calculate fingertip positions.

    //If we wish to calculate hand variables, we need a "hand profile" to tell the Sense Glove our hand lengths
    SGCore::Kinematics::BasicHandModel handModel = SGCore::Kinematics::BasicHandModel::Default(glove->IsRight()); //create a default profile, either left or right.
    //HandPose Example
    SGCore::HandPose handPose;
    if (glove->GetHandPose(handModel, *profile, handPose)) {
        return handPose;
    } else
        throw std::runtime_error("Failed to Get Hand Pose");

}

void GloveData::printGloveInfo(SGCore::SG::SenseGlove *glove) {
    SGCore::SG::SG_GloveInfo model = glove->GetGloveModel();
    std::cout << "---------------------------------------" << std::endl;
    std::cout << (glove->IsRight() ? "Right" : "Left") << " Glove: " << std::endl;
    std::cout << model.ToString(true) << std::endl; //Log some basic information to the user. (true indicates a short notation is desired)
    std::cout << "---------------------------------------" << std::endl;
}

std::vector<SGCore::Kinematics::Vect3D>
GloveData::getTipPositions(SGCore::SG::SenseGlove *glove, SGCore::SG::SG_GlovePose *glovePose,
                           SGCore::HandProfile *profile) {
    return glovePose->CalculateFingerTips(profile->senseGloveProfile); //calculates fingertip position
}

SGCore::HandPose GloveData::getRightHandPose() {
    if (!hasRightGlove()) throw std::runtime_error("No Right Glove Connected!");
    if (!isCalibratedRight()) throw std::runtime_error("Right Glove Not Calibrated!");
    SGCore::SG::SG_GlovePose pose = getPose(m_rightGlove);
    return getHandPose(m_rightGlove, &pose, m_rightProfile);
}

SGCore::HandPose GloveData::getLeftHandPose() {
    if (!hasLeftGlove()) throw std::runtime_error("No Left Glove Connected!");
    if (!isCalibratedLeft()) throw std::runtime_error("Left Glove Not Calibrated!");
    SGCore::SG::SG_GlovePose pose = getPose(m_leftGlove);
    return getHandPose(m_leftGlove, &pose, m_leftProfile);
}
