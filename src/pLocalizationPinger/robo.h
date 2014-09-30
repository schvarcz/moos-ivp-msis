#ifndef ROBO_H
#define ROBO_H

#include <iostream>
#include <vector>
#include "configuracaoes.h"
#include "landmark.h"

using namespace std;

class Hybrid;

class Robo
{
public:
    vector <xyz> mPose;
    vector <xyz> mTruePose;
    vector <xyz> mGyrocompass;
    vector <xyz> mLinearAcceleration;
    vector <xyz> mAngularVelocity;
    vector <xyz> mLinearVelocity;
    vector <double> mDepth;

    clock_t mGPSTime, mLandmarkTime;
    vector <xyz> mGPS;
    vector < vector <Landmark> > mLandmarks;

    clock_t mCurrentTime, mLastEstimation;

    Hybrid *mLocalizationSystem;

    Robo();
    ~Robo();
    void setPose(xyz pose);
    xyz getPose();

    void setTruePose(xyz truePose);
    xyz getTruePose();

    void setAccelerometer(xyz accelerometer);
    xyz getAccelerometer();

    void setLinearVelocity(xyz linearVelocity);
    xyz getLinearVelocity();

    void setDepth(double depth);
    double getDepth();

    void setGyroscope(xyz gyroscope);
    xyz getGyroscope();

    void setGyrocompass(xyz gyrocompass);
    xyz getGyrocompass();

    xyz getDeltaHeading();

    void setLandmarks(vector <Landmark> landmarks);
    vector <Landmark> getLandmarks();

    // TODO: Consider the gps as a point with uncertainty
    void setGPS(xyz gps);
    xyz getGPS();

    bool isGPSTooOld();
    bool isLandmarkTooOld();

    double getElapsedTime();

    std::string toString();
    void findYourself();
};

#include "hybrid.h"

#endif // ROBO_H
