/**
 * \file LocalizationPinger.cpp
 * \brief Classe LocalizationPinger
 * \author Team CISSAU - Veni Vidi Vici (ENSTA Bretagne)
 * \version 0.1
 * \date Jun 5th 2013
 */

#include "LocalizationPinger.h"


using namespace std;

/**
 * \fn
 * \brief Constructeur de l'application MOOS
 */

LocalizationPinger::LocalizationPinger(): 
    pool(50.,50.),
    pool_utm(0.,0.) //width, height
{
    m_iterations = 0;
    m_timewarp   = 1;
    pool_angle = MOOSDeg2Rad(0.0);
}

/**
 * \fn
 * \brief Destructeur de l'instance de l'application
 */

LocalizationPinger::~LocalizationPinger()
{
}

/**
 * \fn
 * \brief Méthode appelée lorsqu'une variable de la MOOSDB est mise à jour
 * N'est appelée que si l'application s'est liée à la variable en question
 */

bool LocalizationPinger::OnNewMail(MOOSMSG_LIST &NewMail)
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

        if( msg.GetKey() == "GPS")
        {
            msg.GetDouble();
        }
        if( msg.GetKey() == "HEADING")
        {
            heading = msg.GetDouble();
        }
        if( msg.GetKey() == "DEPTH")
        {
            depth = msg.GetDouble();
        }
        if( msg.GetKey() == "USV_GPS")
        {
        }
        if( msg.GetKey() == "USV_DISTANCE")
        {
            distance = msg.GetDouble();
        }
    }

    return(true);
}

/**
 * \fn
 * \brief Méthode appelée dès que le contact avec la MOOSDB est effectué
 */

bool LocalizationPinger::OnConnectToServer()
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

bool LocalizationPinger::Iterate()
{

    auv.setLandmarks(landmarksUsados[i]);
    auv.setDeep(depth); //ops! sorry about the name of the method.
    if (i % 400 == 0)
        auv.setGPS(gpsAUV[i]);
    auv.setGyrocompass(heading);

    auv.findYourself();
    m_iterations++;
    m_Comms.Notify("NAV_X",robot.x);
    m_Comms.Notify("NAV_Y",robot.y);
    m_Comms.Notify("NAV_UTM_X",robotUTM.x);
    m_Comms.Notify("NAV_UTM_Y",robotUTM.y);
    return(true);
}

/**
 * \fn
 * \brief Méthode appelée au lancement de l'application
 */

bool LocalizationPinger::OnStartUp()
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

            if(param == "LINEAR_VELOCITY")
                linearVelocity = atoi((char*)value.c_str());

        }
        auv.setLinearVelocity(linearVelocity);
    }

    m_timewarp = GetMOOSTimeWarp();

    RegisterVariables();
    return(true);
}

/**
 * \fn
 * \brief Inscription de l'application à l'évolution de certaines variables de la MOOSDB
 */

void LocalizationPinger::RegisterVariables()
{
    // m_Comms.Register("FOOBAR", 0);
    m_Comms.Register("USV_GPS", 0);
    m_Comms.Register("USV_DISTANCE", 0);
    m_Comms.Register("GPS", 0);
    m_Comms.Register("DEPTH", 0);
    m_Comms.Register("HEADING", 0);
}


