/**
 * \file LocalizationPF.cpp
 * \brief Classe LocalizationPF
 * \author Team CISSAU - Veni Vidi Vici (ENSTA Bretagne)
 * \version 0.1
 * \date Jun 5th 2013
 */

#include "LocalizationPF.h"


using namespace std;

/**
 * \fn
 * \brief Constructeur de l'application MOOS
 */

LocalizationPF::LocalizationPF(): 
    pool(50.,50.),
    pool_utm(0.,0.), //width, height
    m_speed_factor(0.0115),
    m_nav_x(0),m_nav_y(0),
    m_nav_heading(0),
    m_desired_thrust(0)
{
    m_time = MOOSTime();

    m_iterations = 0;
    m_timewarp   = 1;
    pool_angle = MOOSDeg2Rad(0.0);
    sonarImg.create(400,400,CV_8UC1);
    mapping.create(400,400,CV_32FC1);
    
    scale = sonarImg.cols/100.; //max representation in meters
}

/**
 * \fn
 * \brief Destructeur de l'instance de l'application
 */

LocalizationPF::~LocalizationPF()
{
}

/**
 * \fn
 * \brief Méthode appelée lorsqu'une variable de la MOOSDB est mise à jour
 * N'est appelée que si l'application s'est liée à la variable en question
 */

bool LocalizationPF::OnNewMail(MOOSMSG_LIST &NewMail)
{
    MOOSMSG_LIST::iterator p;

    for(p = NewMail.begin() ; p != NewMail.end() ; p++)
    {
        CMOOSMsg &msg = *p;

        #if 0 // Keep these around just for template
        string key   = msg.GetKey();
        string comm  = msg.GetCommunity();
        double dval  = msg.GetDouble();
        string sval  = msg.GetString();
        string msrc  = msg.GetSource();
        double mtime = msg.GetTime();
        bool   mdbl  = msg.IsDouble();
        bool   mstr  = msg.IsString();
        #endif

        if(msg.GetKey() == "DESIRED_THRUST") 
            m_desired_thrust = msg.GetDouble();

        if (msg.GetKey() == "NAV_HEADING")
            m_nav_heading = msg.GetDouble();


        if( msg.GetKey() == "YAW")
        {
            heading_razor = MOOSDeg2Rad(msg.GetDouble());
            double a = MOOSDeg2Rad(-12.6), b = 0.45, c = MOOSDeg2Rad(-10.5);
            heading = heading_razor - ( a*sin(heading_razor+b) + c);

            heading += pool_angle;
        }
        if( msg.GetKey() == "HEADING")
        {
            heading = MOOSDeg2Rad(msg.GetDouble());
            //heading += pool_angle;
        }
        if( msg.GetKey() == "SONAR_RAW_DATA")
        {
            float angle = 0;
            MOOSValFromString(angle, msg.GetString(), "bearing");
            
            vector<unsigned int> scanline;
            int nRows, nCols;
            MOOSValFromString(scanline, nRows, nCols, msg.GetString(), "scanline");

            float ad_interval = 0.25056;
            MOOSValFromString(ad_interval, msg.GetString(), "ad_interval");
            //double scale = 60.0;
            double scale = 4.0;
            double mag_step = scale * ad_interval / 2.0;

            for (double alpha = angle-2.; alpha <angle+2.; alpha+=0.5)
            {
                double cos_b = cos(MOOSDeg2Rad(-alpha) + heading );
                double sin_b = sin(MOOSDeg2Rad(-alpha) + heading );
                for(unsigned int i=0; i<scanline.size();++i)
                {
                    double d = mag_step * i;
                    int y = sin_b*d + sonarImg.cols/2.-0.5;
                    int x = cos_b*d + sonarImg.rows/2.-0.5;
                    if (x>=0 && x<sonarImg.cols && y>=0 && y<sonarImg.rows)
                        sonarImg.at<unsigned char>(y,x) = scanline[i];
                }
            }
        }
        
        if( msg.GetKey() == "SONAR_DISTANCE")
        {
            float angle = 0;
            MOOSValFromString(angle, msg.GetString(), "bearing");

            float ad_interval = 0.25056;
            MOOSValFromString(ad_interval, msg.GetString(), "ad_interval");
            
            float distance = 0.;
            MOOSValFromString(distance, msg.GetString(), "distance");
            
            double mag_step = scale * distance / 2.0;
//            if (angle < 5)
//                sonarImg = Mat::zeros(sonarImg.size(), sonarImg.type());

            for (double alpha = angle-2.; alpha <angle+2.; alpha+=0.5)
            {
                for(double i=mag_step+ad_interval; i>0;i-=ad_interval)
                {
                    double cos_b = cos(MOOSDeg2Rad(-alpha) + heading);
                    double sin_b = sin(MOOSDeg2Rad(-alpha) + heading);
                    int x = cos_b*i + mapping.cols/2.-0.5 + m_nav_x*scale;
                    int y = sin_b*i + mapping.rows/2.-0.5 - m_nav_y*scale;
                    //int x = cos_b*i + sonarImg.cols/2.-0.5;
                    //int y = sin_b*i + sonarImg.rows/2.-0.5;

                    if (x<0 || x>=mapping.cols || y<0 || y>=mapping.rows)
                        continue;
                    float val = mapping.at<float>(y,x);
                    if((i == mag_step) && (mag_step > 10) && (mag_step < 200))
                    {
                        mapping.at<float>(y,x) = std::min(255.,val +7*255./15.);
                        //sonarImg.at<unsigned char>(y,x) = 255;
                    }
                    else
                    {
                        mapping.at<float>(y,x) = std::max(0.,val -1*255./15.);
                    }
                }
                //sonarImg = sonarImg*0.999 -0.2;
            }
        }
    }

    return(true);
}

/**
 * \fn
 * \brief Méthode appelée dès que le contact avec la MOOSDB est effectué
 */

bool LocalizationPF::OnConnectToServer()
{
    // register for variables here
    // possibly look at the mission file?
    // m_MissionReader.GetConfigurationParam("Name", <string>);
    // m_Comms.Register("VARNAME", 0);

    RegisterVariables();
    return(true);
}

/**
 * \fn
 * \brief Méthode appelée automatiquement périodiquement
 * Implémentation du comportement de l'application
 */

bool LocalizationPF::Iterate()
{

    double new_time = MOOSTime();
    double dt = new_time - m_time;
    path.push_back(Point2f(m_nav_x*scale+sonarImg.cols/2.,-m_nav_y*scale +sonarImg.rows/2.));
    m_nav_x += sin(heading)*m_speed_factor*m_desired_thrust*dt;
    m_nav_y += cos(heading)*m_speed_factor*m_desired_thrust*dt;
    //cout << m_nav_x << " - " << m_nav_y << endl;

    m_time = new_time;
    
    Mat final;
    cvtColor(mapping,final,CV_GRAY2RGB);

    for(int i =0; i<path.size();i++)
        circle(final,path.at(i),2,Scalar(255,0,0),-1);

    imshow("Mapping",final);
    imshow("Sonar",sonarImg);
    waitKey(33);
    return(true);
}

/**
 * \fn
 * \brief Méthode appelée au lancement de l'application
 */

bool LocalizationPF::OnStartUp()
{
    setlocale(LC_ALL, "C");
    list<string> sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    if(m_MissionReader.GetConfiguration(GetAppName(), sParams))
    {
        list<string>::iterator p;
        for(p = sParams.begin() ; p != sParams.end() ; p++)
        {
            string original_line = *p;
            string param = stripBlankEnds(toupper(biteString(*p, '=')));
            string value = stripBlankEnds(*p);

            if(param == "POOL_WIDTH")
                pool.width = atof((char*)value.c_str());

            if(param == "POOL_HEIGHT")
                pool.height = atof((char*)value.c_str());

            if(param == "POOL_UTM_X")
                pool_utm.x = atof((char*)value.c_str());

            if(param == "POOL_UTM_Y")
                pool_utm.y = atof((char*)value.c_str());

            if(param == "POOL_ANGLE")
                pool_angle = atof((char*)value.c_str());

            if(param == "SPEED_FACTOR")
                m_speed_factor = atof(value.c_str());

        }
    }
    m_timewarp = GetMOOSTimeWarp();

    RegisterVariables();
    return(true);
}

/**
 * \fn
 * \brief Inscription de l'application à l'évolution de certaines variables de la MOOSDB
 */

void LocalizationPF::RegisterVariables()
{
    // m_Comms.Register("FOOBAR", 0);
    //m_Comms.Register("SONAR_RAW_DATA", 0);
    m_Comms.Register("DESIRED_THRUST",0);
    m_Comms.Register("NAV_HEADING",0);
    m_Comms.Register("NAV_X",0);
    m_Comms.Register("NAV_Y",0);
    m_Comms.Register("DR_SPEED_FACTOR",0);
    m_Comms.Register("SONAR_RAW_DATA", 0);
    m_Comms.Register("SONAR_DISTANCE", 0);
    //m_Comms.Register("YAW", 0);
    m_Comms.Register("HEADING", 0);
}

