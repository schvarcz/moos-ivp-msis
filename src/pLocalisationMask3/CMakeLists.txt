#--------------------------------------------------------
# The CMakeLists.txt for:                       pLocalizationMask
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  LocalizationMask.cpp
  LocalizationMask_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pLocalizationMask ${SRC})

TARGET_LINK_LIBRARIES(pLocalizationMask
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
   opencv_core
   opencv_imgproc
   opencv_highgui
   opencv_ml
   opencv_video
   opencv_features2d
   opencv_calib3d
   opencv_objdetect
   opencv_contrib
   opencv_legacy
   opencv_flann)

