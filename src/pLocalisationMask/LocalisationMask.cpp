/************************************************************/
/**    NAME:                                               **/
/**    FILE: LocalisationMask.cpp                             **/
/**    DATE:                                               **/
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "LocalisationMask.h"

// 210x265 = bassin

using namespace std;

//---------------------------------------------------------
// Constructor

LocalisationMask::LocalisationMask()
{
    w_box = 210;
    h_box = 265;

    best_x_box = 0.;
    best_y_box = 0.;
    best_theta = 0.;

    m_interations = 0;

    startProcess = false;
    startBearing = 5000;
    m_messages = 0;

    sonarImg.create(400,400,CV_8UC1);
//    Mat img_original = imread("./src/pLocalisationMask/tests/3.jpg", CV_LOAD_IMAGE_GRAYSCALE);
//    processImage(img_original);
}

void LocalisationMask::processImage(Mat img_original)
{
  m_interations++;
  cout << "Processing: " << m_interations << endl;
  // Original
  //imshow("Original", img_original);
  
  // Gray
//  Mat img_gray;
//  cvtColor(img_original, img_gray, CV_RGB2GRAY);
//  
//  // Equalize
//  Mat img_equalized;
//  equalizeHist(img_gray, img_equalized);
//  imshow("Equalize", img_equalized);
  
  // Threshold
  Mat img_threshold;
//  threshold(img_equalized, img_threshold, 200, 255, CV_THRESH_BINARY);
  threshold(img_original, img_threshold, 50, 255, CV_THRESH_BINARY);
  imshow("Threshold", img_threshold);
  
  // Reading
  int k = 0;
  int max_nb = 0;
  Point2f robotPos(200,200);
  
  unsigned char *input = (unsigned char*)(img_threshold.data);
  for(int x_box = max(best_x_box-30,0) ; x_box < min(best_x_box+30,img_original.cols / 2) ; x_box+= 10)
  {
    cout << ".." << x_box << " - " << best_x_box << endl;
    for(int y_box = max(best_y_box-30,0) ; y_box < min(best_y_box+30,img_original.rows / 2) ; y_box+=10)
    {
      for(float theta=best_theta-30.;theta<best_theta+30.;theta+=5.)
      //for(float theta=best_theta-90.;theta<best_theta+90.;theta+=5.)
      {
          float angle = (((int)theta)%360)*M_PI/180.;
          
          Point2f pts[4];
          pts[0] = Point2f(x_box,y_box);
          pts[1] = Point2f(x_box,y_box+h_box);
          pts[2] = Point2f(x_box+w_box,y_box+h_box);
          pts[3] = Point2f(x_box+w_box,y_box);
          
          Point2f minPt(400,400), maxPt(0,0);
          for(int index = 0; index<4;index++)
          {
              Point2f pt = translatePt2f(pts[index],robotPos);
              pt = rotatePt2f(pt,angle);
              pts[index] = translatePt2f(pt,Point2f(-robotPos.x,-robotPos.y));
              
              minPt.x = min(pts[index].x,minPt.x);
              minPt.y = min(pts[index].y,minPt.y);
              
              maxPt.x = max(pts[index].x,maxPt.x);
              maxPt.y = max(pts[index].y,maxPt.y);
              
          }
          
          int nb = 0;
          
          LineG lines[4] = {
            LineG(pts[0],pts[1]),
            LineG(pts[3],pts[2]),
            LineG(pts[0],pts[3]),
            LineG(pts[1],pts[2]),
          };
          for(int i = max((int)minPt.x,0) ; i < min((int)maxPt.x, img_original.cols) ; i++)
            for(int j = max((int)minPt.y,0) ; j < min((int)maxPt.y, img_original.rows) ; j++)
            {
                Point2f current(i,j);
                if(
                    (lines[0].isEsq(current) && lines[1].isDir(current)) ||
                    (lines[1].isEsq(current) && lines[0].isDir(current))
                )
                    if(
                        (lines[2].isEsq(current) && lines[3].isDir(current)) ||
                        (lines[3].isEsq(current) && lines[2].isDir(current))
                    )
                        if(input[img_threshold.step * j + i ] !=  0)
                            nb++;
            }
          
          k++;
          
          if(nb > max_nb)
          {
            max_nb = nb;
            
            for (int index=0;index<4;index++)
                corners[index] = pts[index];
            best_x_box = x_box;
            best_y_box = y_box;
            best_theta = ((int)theta)%360;
            if (best_theta > 90 && best_theta < 180)
                best_theta += 180.;
          }
      }
    }
  }
  
  // Rendering
  render(img_original, img_original, "F1", best_x_box, best_y_box, w_box, h_box);
  cout << max_nb << " : " << best_x_box << "," << best_y_box << endl;

  waitKey(20);
}


Point2f LocalisationMask::rotatePt2f(Point2f pt, float angle)
{
    Point2f ret;
    ret.x = pt.x*cos(angle) + pt.y*sin(angle);
    ret.y = -pt.x*sin(angle) +pt.y*cos(angle);
    return ret;
}

Point2f LocalisationMask::translatePt2f(Point2f pt, Point2f center)
{
    Point2f ret;
    ret.x = pt.x - center.x;
    ret.y = pt.y - center.y;
    return ret;
}

void LocalisationMask::render(Mat img_original, Mat img_to_render, string title, int best_x_box, int best_y_box, int w_box, int h_box)
{
  Mat img_final;
  cvtColor(img_original,img_final,CV_GRAY2RGB);
  //img_to_render.copyTo(img_final);
  line(img_final, 
        corners[0], 
        corners[1], 
        Scalar(0, 255, 255));
  line(img_final,  
        corners[2], 
        corners[3], 
        Scalar(0, 255, 255));
  line(img_final,  
        corners[0], 
        corners[3], 
        Scalar(0, 255, 255));
  line(img_final, 
        corners[1], 
        corners[2], 
        Scalar(0, 255, 255));
  imshow(title, img_final);
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool LocalisationMask::OnNewMail(MOOSMSG_LIST &NewMail)
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

        if( msg.GetKey() == "HEADING")
        {
            heading = MOOSDeg2Rad(msg.GetDouble());
        }
        if( msg.GetKey() == "SONAR_RAW_DATA")
        {
            float angle = 0;
            MOOSValFromString(angle, msg.GetString(), "bearing");
            
            if ((abs(startBearing-angle) <2) && (m_messages>40))
                startProcess = true;
            else
                m_messages++;
            if (startBearing == 5000)
                startBearing = angle;
            vector<unsigned int> scanline;
            int nRows, nCols;
            MOOSValFromString(scanline, nRows, nCols, msg.GetString(), "scanline");

            float ad_interval = 0.25056;
            MOOSValFromString(ad_interval, msg.GetString(), "ad_interval");
            //double scale = 60.0;
            double scale = 4.0;
            double mag_step = scale * ad_interval / 2.0;

            sonarImg = sonarImg*0.999 -0.1;

            for (double alpha = angle-2.; alpha <angle+2.; alpha+=0.5)
            {
                double cos_b = cos(MOOSDeg2Rad(-alpha) + heading );
                double sin_b = sin(MOOSDeg2Rad(-alpha) + heading );
                for(unsigned int i=0; i<scanline.size();++i)
                {
                    double d = mag_step * i;
                    int x = sin_b*d + sonarImg.cols/2.-0.5;
                    int y = cos_b*d + sonarImg.rows/2.-0.5;
                    if (x>=0 && x<sonarImg.cols && y>=0 && y<sonarImg.rows)
                        sonarImg.at<unsigned char>(x,y) = scanline[i];
                }
                imshow("Sonar",sonarImg);
                //waitKey(20);
            }
        }

        /*else */if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
          reportRunWarning("Unhandled Mail: " + key);
        }

    return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool LocalisationMask::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool LocalisationMask::Iterate()
{
    AppCastingMOOSApp::Iterate();

    if (startProcess)
        processImage(sonarImg);

    AppCastingMOOSApp::PostReport();
    return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool LocalisationMask::OnStartUp()
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
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables

void LocalisationMask::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  m_Comms.Register("SONAR_RAW_DATA", 0);
  m_Comms.Register("HEADING", 0);
  //Register("NAV_X", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool LocalisationMask::buildReport() 
{
  //m_msgs << "Logs file name: " << m_logs_folder << "/" << m_logs_file_name << endl;
  return(true);
}
