//
// Created by orange on 2/9/21.
//

#include "../includes/DataCaptureApp.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void DataCaptureApp::setup()
{
    auto lambert = gl::ShaderDef().lambert().color();
    gl::GlslProgRef shader = gl::getStockShader(lambert);
    mPalm = gl::Batch::create(geom::Cube().size(10.0, 4.0, 15.0), shader);

    mThumb1 = gl::Batch::create(geom::Cube().size(FINGERW, FINGERH, THUMB1), shader);
    mThumb2 = gl::Batch::create(geom::Cube().size(FINGERW-0.1, FINGERH-0.1, THUMB2), shader);
    mIndex1 = gl::Batch::create(geom::Cube().size(FINGERW, FINGERH, LENGTH1), shader);
    mIndex2 = gl::Batch::create(geom::Cube().size(FINGERW-0.1, FINGERH-0.1, LENGTH2), shader);
    mIndex3 = gl::Batch::create(geom::Cube().size(FINGERW-0.2, FINGERH-0.2, LENGTH3), shader);
    mMiddle1 = gl::Batch::create(geom::Cube().size(FINGERW, FINGERH, LENGTH1), shader);
    mMiddle2 = gl::Batch::create(geom::Cube().size(FINGERW-0.1, FINGERH-0.1, LENGTH2), shader);
    mMiddle3 = gl::Batch::create(geom::Cube().size(FINGERW-0.2, FINGERH-0.2, LENGTH3), shader);
    mRing1 = gl::Batch::create(geom::Cube().size(FINGERW, FINGERH,LENGTH1), shader);
    mRing2 = gl::Batch::create(geom::Cube().size(FINGERW-0.1, FINGERH-0.1, LENGTH2), shader);
    mRing3 = gl::Batch::create(geom::Cube().size(FINGERW-0.2, FINGERH-0.2, LENGTH3), shader);
    mPinky1 = gl::Batch::create(geom::Cube().size(FINGERW, FINGERH, LENGTH1), shader);
    mPinky2 = gl::Batch::create(geom::Cube().size(FINGERW-0.1, FINGERH-0.1, LENGTH2), shader);
    mPinky3 = gl::Batch::create(geom::Cube().size(FINGERW-0.2, FINGERH-0.2, LENGTH3), shader);

}


void DataCaptureApp::update()
{
    AppBase::update();
    mRotation *= rotate( toRadians( 0.2f ), normalize( vec3( 0, 1, 0 ) ) );
}

void DataCaptureApp::draw()
{
    double PI = 3.14159265;
    gl::clear();
    gl::enableDepthRead();
    gl::enableDepthWrite();

    gl::setMatrices(mCam);

    gl::ScopedModelMatrix modelScope;
    gl::multModelMatrix( mRotation );

    AppBase::draw();

    // gl::drawCoordinateFrame(10.0);
    gl::translate(0, 0, -4.5);
    mPalm->draw();



    gl::translate(0,0,7.5); // move to front of palm

    gl::translate(3.75, 0, LENGTH1 / 2.0); // move to index position
    mIndex1->draw();
    gl::translate(0, -(LENGTH2 / 2.0) * sin(jointAngles[1][3] * PI / 180.0), LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[1][3] * PI / 180.0));
    gl::rotate(jointAngles[1][3] * PI / 180, 1, 0 ,0);
    mIndex2->draw();
    gl::translate(0, -(LENGTH3 / 2.0) * sin(jointAngles[1][4] * PI / 180.0), LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[1][4] * PI / 180.0));
    gl::rotate(jointAngles[1][4] * PI / 180, 1, 0 ,0);
    mIndex3->draw();

    gl::rotate(-jointAngles[1][4] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH3 / 2.0) * sin(jointAngles[1][4] * PI / 180.0), -(LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[1][4] * PI / 180.0)));
    gl::rotate(-jointAngles[1][3] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH2 / 2.0) * sin(jointAngles[1][3] * PI / 180.0), -(LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[1][3] * PI / 180.0)));

    gl::translate(-2.5, 0, 0);
    mMiddle1->draw();
    gl::translate(0, -(LENGTH2 / 2.0) * sin(jointAngles[2][3] * PI / 180.0), LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[2][3] * PI / 180.0));
    gl::rotate(jointAngles[2][3] * PI / 180, 1, 0 ,0);
    mMiddle2->draw();
    gl::translate(0, -(LENGTH3 / 2.0) * sin(jointAngles[2][4] * PI / 180.0), LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[2][4] * PI / 180.0));
    gl::rotate(jointAngles[2][4] * PI / 180, 1, 0 ,0);
    mMiddle3->draw();

    gl::rotate(-jointAngles[2][4] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH3 / 2.0) * sin(jointAngles[2][4] * PI / 180.0), -(LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[2][4] * PI / 180.0)));
    gl::rotate(-jointAngles[2][3] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH2 / 2.0) * sin(jointAngles[2][3] * PI / 180.0), -(LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[2][3] * PI / 180.0)));

    gl::translate(-2.5, 0, 0);
    mRing1->draw();
    gl::translate(0, -(LENGTH2 / 2.0) * sin(jointAngles[3][3] * PI / 180.0), LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[3][3] * PI / 180.0));
    gl::rotate(jointAngles[3][3] * PI / 180, 1, 0 ,0);
    mRing2->draw();
    gl::translate(0, -(LENGTH3 / 2.0) * sin(jointAngles[3][4] * PI / 180.0), LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[3][4] * PI / 180.0));
    gl::rotate(jointAngles[3][4] * PI / 180, 1, 0 ,0);
    mRing3->draw();

    gl::rotate(-jointAngles[3][4] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH3 / 2.0) * sin(jointAngles[3][4] * PI / 180.0), -(LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[3][4] * PI / 180.0)));
    gl::rotate(-jointAngles[3][3] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH2 / 2.0) * sin(jointAngles[3][3] * PI / 180.0), -(LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[3][3] * PI / 180.0)));

    gl::translate(-2.5, 0, 0);
    mPinky1->draw();
    gl::translate(0, -(LENGTH2 / 2.0) * sin(jointAngles[4][3] * PI / 180.0), LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[4][3] * PI / 180.0));
    gl::rotate(jointAngles[4][3] * PI / 180, 1, 0 ,0);
    mPinky2->draw();
    gl::translate(0, -(LENGTH3 / 2.0) * sin(jointAngles[4][4] * PI / 180.0), LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[4][4] * PI / 180.0));
    gl::rotate(jointAngles[4][4] * PI / 180, 1, 0 ,0);
    mPinky3->draw();

    gl::rotate(-jointAngles[4][4] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH3 / 2.0) * sin(jointAngles[4][4] * PI / 180.0), -(LENGTH2 / 2.0 + (LENGTH3 / 2.0) * cos(jointAngles[4][4] * PI / 180.0)));
    gl::rotate(-jointAngles[4][3] * PI / 180, 1, 0 ,0);
    gl::translate(0, (LENGTH2 / 2.0) * sin(jointAngles[4][3] * PI / 180.0), -(LENGTH1 / 2.0 + (LENGTH2 / 2.0) * cos(jointAngles[4][3] * PI / 180.0)));

}

CINDER_APP( DataCaptureApp, RendererGl( RendererGl::Options().msaa( 8 ) ), []( App::Settings *settings ) {
    settings->setWindowSize(1200, 800);
    settings->setFrameRate(60.0f);
} )