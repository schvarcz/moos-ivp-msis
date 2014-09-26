/**
 * \file pBuoyDetection.cpp
 * \brief Classe pBuoyDetection
 * \author Team CISSAU - Veni Vidi Vici (ENSTA Bretagne)
 * \version 0.1
 * \date Jun 5th 2013
 */

#include "pBuoyDetection.h"

using namespace std;

/**
 * \fn
 * \brief Constructeur de l'application MOOS
 */

pBuoyDetection::pBuoyDetection() : start_record(true), show_process(true), message_name("Buoy")
{
    m_iterations = 0;
    m_timewarp   = 1;
}

/**
 * \fn
 * \brief Destructeur de l'instance de l'application
 */

pBuoyDetection::~pBuoyDetection()
{
}

/**
 * \fn
 * \brief Méthode appelée lorsqu'une variable de la MOOSDB est mise à jour
 * N'est appelée que si l'application s'est liée à la variable en question
 */

bool pBuoyDetection::OnNewMail(MOOSMSG_LIST &NewMail)
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

        if( msg.GetKey() == image_name)
        {
            start_record = true;
            memcpy(img.data, msg.GetBinaryData(), img.rows*img.step);
        }
    }

    return(true);
}

/**
 * \fn
 * \brief Méthode appelée dès que le contact avec la MOOSDB est effectué
 */

bool pBuoyDetection::OnConnectToServer()
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

bool pBuoyDetection::Iterate()
{
    m_iterations++;
    if (start_record)
    {
        //img = imread("/home/schvarcz/Desktop/Missão Eurathlon/10th/PICS_2014-09-24_16-57-37/BOTTOM_16_58_06.jpg");
        Mat imgHSV,imgThr,imgThr2,eqHSV;
        img = img(Rect(10,10,img.cols-20,img.rows-20));
        blur(img,img,Size(3,3));
        blur(img,img,Size(3,3));
        cvtColor(img,imgHSV,CV_RGB2HSV);
        vector<Mat> channels;
        split(imgHSV,channels);
        for (int i =0; i<channels.size();i++)
            equalizeHist(channels[i],channels[i]);
        merge(channels,eqHSV);
        
        inRange(imgHSV,Scalar(95,100,0),Scalar(200,200,255),imgThr);
        inRange(eqHSV,Scalar(250,240,250),Scalar(255,255,255),imgThr2);
        
        Moments m1 = moments(imgThr);
        int found = 0;
        if (m1.m00 != 0)
        {
            found += 1;
            if (show_process)
            {
                Point center1(m1.m10/m1.m00,m1.m01/m1.m00);
                circle(img,center1,5,Scalar(0,0,255));
            }
        }
        Moments m2 = moments(imgThr2);
        if (m2.m00 != 0)
        {
            found += 2;
            if (show_process)
            {
                Point center2(m2.m10/m2.m00,m2.m01/m2.m00);
                circle(img,center2,5,Scalar(255,0,0));
            }
        }

        switch(found)
        {
            case 1:
                m_Comms.Notify(message_name, "method 1");
                break;
            case 2:
                m_Comms.Notify(message_name, "method 2");
                break;
            case 3:
                m_Comms.Notify(message_name, "method 1 and 2");
                break;
        }

        if(show_process)
        {
            imshow("Original",img);
            imshow("Result",imgThr);
            imshow("Result",imgThr);
            imshow("Result2",imgThr2);
            imshow("imghsv",imgHSV);
            imshow("Heq",eqHSV);
            waitKey(20);
        }
    }
    return(true);
}

/**
 * \fn
 * \brief Méthode appelée au lancement de l'application
 */

bool pBuoyDetection::OnStartUp()
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

            if(param == "VARIABLE_IMAGE_NAME")
            {
                image_name = value;
            }
            if(param == "VARIABLE_WHEN_FOUND")
            {
                message_name = value;
            }
            if(param == "SHOW_PROCESS")
            {
                show_process = (value == "true");
            }


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

void pBuoyDetection::RegisterVariables()
{
    // m_Comms.Register("FOOBAR", 0);
    m_Comms.Register(image_name, 0);
}

