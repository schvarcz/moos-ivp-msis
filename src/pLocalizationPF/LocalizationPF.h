/**
 * \file LocalizationPF.h
 * \brief Classe LocalizationPF
 * \author Team CISSAU - Veni Vidi Vici (ENSTA Bretagne)
 * \version 0.1
 * \date Jun 5th 2013
 */

#ifndef LocalizationPF_HEADER
#define LocalizationPF_HEADER

#include <iterator>
#include "MBUtils.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MOOS/libMOOS/App/MOOSApp.h"
#include <math.h>

using namespace std;
using namespace cv;

class LocalizationPF : public CMOOSApp
{
    public:
        LocalizationPF();
        ~LocalizationPF();

    protected:
        bool OnNewMail(MOOSMSG_LIST &NewMail);
        bool Iterate();
        bool OnConnectToServer();
        bool OnStartUp();
        void RegisterVariables();

    private: // Configuration variables
        Point pool_utm;
        Size pool;
        Mat sonarImg, mapping;
        double pool_angle;
        double heading;
        double heading_razor;
        double scale;

    private: // State variables
        double m_nav_x, m_nav_y;
        double m_nav_heading;
        double m_desired_thrust;
     	double m_speed_factor;
        double m_time;
        vector<Point2f> path;


    private: // State variables
        unsigned int    m_iterations;
        double            m_timewarp;
};

#endif 
