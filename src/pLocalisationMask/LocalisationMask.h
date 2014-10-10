/************************************************************/
/**    NAME:                                               **/
/**    FILE: LocalisationMask.h                               **/
/**    DATE: December 29th, 1963                           **/
/************************************************************/

#ifndef LocalisationMask_HEADER
#define LocalisationMask_HEADER

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

using namespace std;
using namespace cv;

class LocalisationMask : public AppCastingMOOSApp
{
  public:
    LocalisationMask();
    ~LocalisationMask() {};

  protected: // Standard MOOSApp functions to overload  
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();
    void processImage(Mat img_original);

  protected: // Standard AppCastingMOOSApp function to overload 
  
    Point2f corners[4];
    Point2f rotatePt2f(Point2f pt, float angle);
    Point2f translatePt2f(Point2f pt, Point2f center);
    bool buildReport();
    void render(Mat img_original, Mat img_to_render, string title, int best_x_box, int best_y_box, int w_box, int h_box);

  protected:
    void RegisterVariables();

    int w_box, h_box;
    unsigned int m_interations, m_messages;
    bool startProcess;
    
    int best_x_box, best_y_box, best_theta;
    float heading, startBearing;
    Mat sonarImg;

  private:
};

struct LineG {
    float a, b, c;
    bool flag;
    LineG(): flag(true){};
    LineG(Point2f p1, Point2f p2){
        if(p1.x == p2.x)
        {
            a = 1.0;
            b = 0.0;
            c = -p1.x;
            flag = p2.y < p1.y;
        }
        else
        {
            a = -(p2.y - p1.y)/(p2.x - p1.x);
            b = 1.0;
            c = -(a*p1.x + b*p1.y);
            flag = p2.x > p1.x;
        }
    };
    bool isEsq(Point2f p) {
        if (a*p.x+b*p.y+c >= 0)
            return flag;
        return !flag;
    };
    bool isDir(Point2f p) {
        if (a*p.x+b*p.y+c <= 0)
            return flag;
        return !flag;
    };
  
};

#endif 
