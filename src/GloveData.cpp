//
// Created by orange on 2/9/21.
//

#include "GloveData.h"
/*
A console application demonstrating the intended use of a SenseGlove interface.
Used to compile the programs in the SGCoreCpp/test folder.
*/

#include <iostream> //Output to console
#include <fstream>
#include <sys/stat.h>

#include <thread>  //To pause the main() while vibrating
#include <chrono>  //To pause the thread for std::chrono::seconds

#include "Library.h"	// Access library details
#include "DeviceList.h" // Access devices and communication state
#include "SenseGlove.h" // SenseGlove interface through which we access data.

GloveData::GloveData(SGCore::HandProfile *profileLeft, SGCore::HandProfile *profileRight) {
    SGCore::SG::SenseGlove glove1;
    if (!SGCore::DeviceList::SenseCommRunning())
        throw std::runtime_error("Sense Comm not running.");
    if (!SGCore::SG::SenseGlove::GetSenseGlove(glove1))
        throw std::runtime_error("No Sense Gloves Detected!");
    else {
        if (SGCore::SG::SenseGlove::GetSenseGlove(false, m_leftGlove)) hasLeft = true;
        if (SGCore::SG::SenseGlove::GetSenseGlove(true, m_rightGlove)) hasRight = true;
    }
    if (profileLeft != nullptr)
        m_leftProfile = *profileLeft;
    if (profileRight != nullptr)
        m_rightProfile = *profileRight;

    if (hasRightGlove()) printGloveInfo(&m_rightGlove);
    if (hasLeftGlove()) printGloveInfo(&m_leftGlove);
}

void GloveData::calibrateLeft() {
    if (hasLeftGlove()) {
        calibLeft = true;
        m_leftProfile = calibration(&m_leftGlove);
    } else
        throw std::runtime_error("No Left Glove Connected!");
}

void GloveData::calibrateRight() {
    if (hasRightGlove()) {
        calibRight = true;
        m_rightProfile = calibration(&m_rightGlove);
    } else
        throw std::runtime_error("No Right Glove Connected!");
}

bool GloveData::hasRightGlove() {
    return hasRight;
}

bool GloveData::hasLeftGlove() {
    return hasLeft;
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
    return calibLeft;
}

bool GloveData::isCalibratedRight() {
    return calibRight;
}

SGCore::SG::SG_GlovePose GloveData::getPose(SGCore::SG::SenseGlove *glove) {
    //Retrieving Glove Pose: The position / rotation of the glove, as well as its sensor angles placed in the right direction.
    SGCore::SG::SG_GlovePose glovePose;
    if (glove->GetGlovePose(glovePose))
        return glovePose;
    else
        throw std::runtime_error("Failed to get glove pose.");
}

void GloveData::getHandPose(SGCore::SG::SenseGlove *glove, SGCore::HandProfile *profile, SGCore::HandPose &handpose) {
    //As an example, lets calculate fingertip positions.

    //If we wish to calculate hand variables, we need a "hand profile" to tell the Sense Glove our hand lengths
    SGCore::Kinematics::BasicHandModel handModel = SGCore::Kinematics::BasicHandModel::Default(glove->IsRight()); //create a default profile, either left or right.
    //HandPose Example
    if (!glove->GetHandPose(handModel, *profile, handpose))
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

void GloveData::getRightHandPose(SGCore::HandPose &handpose) {
    if (!hasRightGlove()) throw std::runtime_error("No Right Glove Connected!");
    if (!isCalibratedRight()) throw std::runtime_error("Right Glove Not Calibrated!");
    pose = getPose(&m_rightGlove);
    getHandPose(&m_rightGlove, &m_rightProfile, handpose);
}

void GloveData::getLeftHandPose(SGCore::HandPose &handpose) {
    if (!hasLeftGlove()) throw std::runtime_error("No Left Glove Connected!");
    if (!isCalibratedLeft()) throw std::runtime_error("Left Glove Not Calibrated!");
    pose = getPose(&m_leftGlove);
    getHandPose(&m_leftGlove, &m_leftProfile, handpose);
}

std::vector<SGCore::Kinematics::Vect3D> GloveData::getRightHandTipPositions() {
    if (!hasRightGlove()) throw std::runtime_error("No Right Glove Connected!");
    if (!isCalibratedRight()) throw std::runtime_error("Right Glove Not Calibrated!");
    pose = getPose(&m_rightGlove);
    return getTipPositions(&m_rightGlove, &pose, &m_rightProfile);
}

void GloveData::saveRightProfile(std::string file) {
    saveProfile(file, &m_rightProfile);
}

void GloveData::saveLeftProfile(std::string file) {
    saveProfile(file, &m_leftProfile);
}

void GloveData::saveProfile(std::string file, SGCore::HandProfile *profile) {
    std::ofstream outfile;
    outfile.open(file);
    outfile << profile->Serialize();
    outfile.close();
}

bool GloveData::loadLeftProfile(std::string file) {
    if (loadProfile(file, m_leftProfile)) {
        calibLeft = true;
        return true;
    }
    return false;
}

bool GloveData::loadRightProfile(std::string file) {
    std::cout<< m_rightProfile.Serialize() <<std::endl;
    if (loadProfile(file, m_rightProfile)) {
        calibRight = true;
        std::cout<< m_rightProfile.Serialize() <<std::endl;
        return true;
    }
    return false;
}

bool GloveData::loadProfile(std::string file, SGCore::HandProfile &profile) {
    struct stat buf;
    if (stat(file.c_str(), &buf) != -1) {
        std::ifstream outfile;
        outfile.open(file);
        std::string profile_str;
        outfile >> profile_str;
        outfile.close();
        profile = SGCore::HandProfile::Deserialize(profile_str);
        return true;
    }
    return false;
}

