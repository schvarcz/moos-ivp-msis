/**
 * \file LocalizationPinger.h
 * \brief Classe LocalizationPinger
 * \author Team CISSAU - Veni Vidi Vici (ENSTA Bretagne)
 * \version 0.1
 * \date Jun 5th 2013
 */

#ifndef LocalizationPinger_HEADER
#define LocalizationPinger_HEADER

#include <iterator>
#include "MBUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MOOS/libMOOS/App/MOOSApp.h"
#include "robo.h"
#include "hybrid.h"


using namespace std;

class LocalizationPinger : public CMOOSApp
{
    public:
        LocalizationPinger();
        ~LocalizationPinger();

    protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();

    private: // Configuration variables
        Point2f pool_utm, robot, robotUTM;
        double pool_angle;
        double depth, heading, distance, linearVelocity;
        
        Robo auv;

    private: // State variables
        unsigned int    m_iterations;
        double            m_timewarp;
};

#endif 
