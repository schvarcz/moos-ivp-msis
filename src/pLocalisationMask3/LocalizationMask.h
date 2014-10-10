/************************************************************/
/**    NAME:                                               **/
/**    FILE: LocalizationMask.h                            **/
/**    DATE: December 29th, 1963                           **/
/************************************************************/

#ifndef LocalizationMask_HEADER
#define LocalizationMask_HEADER

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

using namespace std;
using namespace cv;

class LocalizationMask : public AppCastingMOOSApp
{
  public:
    LocalizationMask();
    ~LocalizationMask() {};

  protected: // Standard MOOSApp functions to overload  
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

  protected: // Standard AppCastingMOOSApp function to overload 
    bool buildReport();
    void render(Mat img_original, Mat img_to_render, string title, int best_x_box, int best_y_box, int w_box, int h_box);

  protected:
    void registerVariables();

  private:
};

#endif 
