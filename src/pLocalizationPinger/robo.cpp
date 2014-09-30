#include "robo.h"

Robo::Robo(): mLastEstimation(NULL),
    mGPSTime(NULL),
    mLandmarkTime(NULL)
{
    mLocalizationSystem = new Hybrid(this);
}

Robo::~Robo()
{
    cout << "Deleting robot" << endl;
    delete mLocalizationSystem;
}

void Robo::setPose(xyz pose)
{
    this->mPose.push_back(pose);
}

xyz Robo::getPose()
{
    return this->mPose.back();
}

void Robo::setTruePose(xyz truePose)
{
    this->mTruePose.push_back(truePose);
}

xyz Robo::getTruePose()
{
    return this->mTruePose.back();
}

void Robo::setAccelerometer(xyz accelerometer)
{
    this->mLinearAcceleration.push_back(accelerometer);
}

xyz Robo::getAccelerometer()
{
    return this->mLinearAcceleration.back();
}

void Robo::setLinearVelocity(xyz linearVelocity)
{
    this->mLinearVelocity.push_back(linearVelocity);
}

xyz Robo::getLinearVelocity()
{
    return this->mLinearVelocity.back();
}

void Robo::setDepth(double depth)
{
    this->mDepth.push_back(depth);
}

double Robo::getDepth()
{
    return this->mDepth.back();
}

void Robo::setGyroscope(xyz gyroscope)
{
    this->mAngularVelocity.push_back(gyroscope);
}

xyz Robo::getGyroscope() //Euler
{
    return this->mAngularVelocity.back();
}

void Robo::setGyrocompass(xyz gyrocompass)
{
    this->mGyrocompass.push_back(gyrocompass);
}

xyz Robo::getGyrocompass() //Euler
{
    return this->mGyrocompass.back();
}

xyz Robo::getDeltaHeading() //Euler
{
    xyz ret;

    if (this->mGyrocompass.size() > 1)
    {
        xyz curGyrocompass = this->mGyrocompass.back();
        xyz lastGyrocompass = this->mGyrocompass[this->mGyrocompass.size()-2];

        ret.x = curGyrocompass.x - lastGyrocompass.x;
        ret.y = curGyrocompass.y - lastGyrocompass.y;
        ret.z = curGyrocompass.z - lastGyrocompass.z;
    }

    return ret;
}

void Robo::setGPS(xyz gps)
{
    mGPSTime = mCurrentTime;
    this->mGPS.push_back(gps);
}

bool Robo::isGPSTooOld()
{
    clock_t elapsedTime = double(mCurrentTime - mGPSTime) / CLOCKS_PER_SEC;
    return elapsedTime > MAX_TIME_FOR_OLD_READING || mGPSTime == NULL;
}

xyz Robo::getGPS()
{
    if (this->isGPSTooOld())
        return xyz();

    return this->mGPS.back();// aceleração x, y, z e tempo
}

void Robo::setLandmarks(vector <Landmark> landmarks)
{
    mLandmarkTime = mCurrentTime;
    this->mLandmarks.push_back(landmarks);
}

bool Robo::isLandmarkTooOld()
{
    clock_t elapsedTime = double(mCurrentTime - mLandmarkTime) / CLOCKS_PER_SEC;
    return elapsedTime > MAX_TIME_FOR_OLD_READING || mLandmarkTime == NULL;
}

vector <Landmark> Robo::getLandmarks()
{
    if (this->isLandmarkTooOld())
        return vector <Landmark>();
    return this->mLandmarks.back();
}

/*
getCamera(){//3 imagens

}

*/

double Robo::getElapsedTime()
{
    double elapsedTime = 0.;

    if (mLastEstimation != NULL)
        elapsedTime = double(mCurrentTime - mLastEstimation) / CLOCKS_PER_SEC;

    return elapsedTime;
}

std::string Robo::toString()
{
    std::ostringstream oss;

    xyz accelerometer = this->getAccelerometer();
    xyz gps = this->getGPS();
    xyz truePose = this->getTruePose();
    xyz gyrocompass = this->getGyrocompass();
    xyz gyroscope = this->getGyroscope();
    xyz linearVelocity = this->getLinearVelocity();

    oss
            << truePose.x       << ", " << truePose.y       << ", " << truePose.z       << ", " << endl
            << gyrocompass.x    << ", " << gyrocompass.y    << ", " << gyrocompass.z    << ", " << endl
            << gps.x            << ", " << gps.y            << ", " << gps.z            << ", " << double(mCurrentTime - mGPSTime) / CLOCKS_PER_SEC << ", " << endl
            << gyroscope.x      << ", " << gyroscope.y      << ", " << gyroscope.z      << ", " << endl
            << linearVelocity.x << ", " << linearVelocity.y << ", " << linearVelocity.z << ", " << endl
            << accelerometer.x  << ", " << accelerometer.y  << ", " << accelerometer.z  << ", " << endl
            << this->getDepth();

    return oss.str();

}

void Robo::findYourself()
{
    //mCurrentTime = clock();
    mLocalizationSystem->findWhereIAm();
    mLastEstimation = mCurrentTime;
}
