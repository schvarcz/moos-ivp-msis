/************************************************************/
/**    NAME:                                               **/
/**    FILE: LocalizationMask.cpp                          **/
/**    DATE:                                               **/
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "LocalizationMask.h"

// 210x265 = bassin

using namespace std;

//---------------------------------------------------------
// Constructor

LocalizationMask::LocalizationMask()
{
  int w_box = 210;
  int h_box = 265;//265;
  
  // Original
  Mat img_original = imread("./tests/0.jpg", 1);
  //imshow("Original", img_original);
  
  // Original_gray
  Mat img_original_gray = imread("./tests/0.jpg", 0);
  //imshow("Original", img_original);
  
  // Gray
  /*Mat img_gray;
  cvtColor(img_original, img_gray, CV_RGB2GRAY);*/
  //imshow("Gray", img_gray);
  
  // Equalize
  /*Mat img_equalized;
  equalizeHist(img_gray, img_equalized);
  imshow("Equalize", img_equalized);*/
  
  // Threshold
  Mat img_threshold = Mat::zeros(Size(img_original_gray.rows, img_original_gray.cols), CV_8U);
  threshold(img_original_gray, img_threshold, 50, 255, CV_THRESH_BINARY);
  imshow("Threshold", img_threshold);
  
  // Reading
  int max_nb = 0;
  int best_x_box = 0, best_y_box = 0;
  
  unsigned char *input = (unsigned char*)(img_threshold.data);
  
  for(int x_box = 0 ; x_box < img_original_gray.cols / 2 ; x_box++)
  {
    if(x_box % 2 == 0) // more fast...
      continue;
    
    cout << ".." << x_box << endl;
    for(int y_box = 0 ; y_box < img_original_gray.rows / 2 ; y_box++)
    {
      if(y_box % 2 == 0) // more fast...
        continue;
      
      int nb = 0;
      for(int i = x_box ; i < min(x_box + w_box, img_original_gray.cols) ; i++)
        for(int j = y_box ; j < min(y_box + h_box, img_original_gray.rows) ; j++)
        {
          if(input[img_threshold.step * j + i ] != 0)
            nb++;
        }
      
      if(nb > max_nb)
      {
        max_nb = nb;
        best_x_box = x_box;
        best_y_box = y_box;
      }
    }
  }
  
  // Rendering
  render(img_original, img_original, "F1", best_x_box, best_y_box, w_box, h_box);
  cout << max_nb << " : " << best_x_box << "," << best_y_box << endl;

  while(waitKey(25) != 10);
  cout << "\tEnd constructor" << endl;
}

void LocalizationMask::render(Mat img_original, Mat img_to_render, string title, int best_x_box, int best_y_box, int w_box, int h_box)
{
  Mat img_final(img_original);
  img_to_render.copyTo(img_final);
  line(img_final, 
        Point(best_x_box, best_y_box), 
        Point(best_x_box + w_box, best_y_box), 
        Scalar(0, 255, 255));
  line(img_final, 
        Point(best_x_box, best_y_box + h_box), 
        Point(best_x_box + w_box, best_y_box + h_box), 
        Scalar(0, 255, 255));
  line(img_final, 
        Point(best_x_box + w_box, best_y_box), 
        Point(best_x_box + w_box, best_y_box + h_box), 
        Scalar(0, 255, 255));
  line(img_final, 
        Point(best_x_box, best_y_box), 
        Point(best_x_box, best_y_box + h_box),
        Scalar(0, 255, 255));
  imshow(title, img_final);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool LocalizationMask::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    
    #if 0 // Keep these around just for template
      string comm  = msg.GetCommunity();
      double dval  = msg.GetDouble();
      string sval  = msg.GetString(); 
      string msrc  = msg.GetSource();
      double mtime = msg.GetTime();
      bool   mdbl  = msg.IsDouble();
      bool   mstr  = msg.IsString();
    #endif
    
    /*if(key == "NAV_X") 
      m_nav_x = msg.GetDouble();*/

    /*else */if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool LocalizationMask::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool LocalizationMask::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  
  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool LocalizationMask::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++)
  {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    
    /*if(param == "LOGS_FOLDER")
    {
      handled = true;
      m_logs_folder = value;
    }
    
    if(param == "LOGS_FILE_NAME")
    {
      handled = true;
      m_logs_file_name = value;
    }*/handled = true;
    
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void LocalizationMask::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  //Register("NAV_X", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool LocalizationMask::buildReport() 
{
  //m_msgs << "Logs file name: " << m_logs_folder << "/" << m_logs_file_name << endl;
  return(true);
}
