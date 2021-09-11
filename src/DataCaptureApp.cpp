//
// Created by orange on 2/9/21.
//

#include "../includes/DataCaptureApp.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void DataCaptureApp::setup() {
    setupGraphics();

    ImGui::Initialize();
}

void DataCaptureApp::setupGraphics()
{

}

void DataCaptureApp::update() {
    AppBase::update();
}

void DataCaptureApp::draw() {
    AppBase::draw();
}

void DataCaptureApp::drawBackground() {

}

void DataCaptureApp::drawDebug() {

}

void DataCaptureApp::drawInfo() {

}


CINDER_APP( DataCaptureApp, RendererGl( RendererGl::Options().msaa( 8 ) ), []( App::Settings *settings ) {
    settings->setWindowSize(1200, 800);
    settings->setFrameRate(60.0f);
} )