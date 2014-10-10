#--------------------------------------------------------
# The CMakeLists.txt for:                       pLocalisationMask
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  LocalisationMask.cpp
  LocalisationMask_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pLocalisationMask ${SRC})

TARGET_LINK_LIBRARIES(pLocalisationMask
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
   opencv_legacy
   opencv_flann)

