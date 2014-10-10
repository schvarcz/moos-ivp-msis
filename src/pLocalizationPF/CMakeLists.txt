#--------------------------------------------------------
# The CMakeLists.txt for:                       pLocalizationPF
# Author(s):                              Équipe Ciscrea - Veni Vidi Vici
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

SET(SRC
  LocalizationPF.cpp
  LocalizationPF_Info.cpp
  main.cpp
)



ADD_EXECUTABLE(pLocalizationPF ${SRC})

TARGET_LINK_LIBRARIES(pLocalizationPF
   MOOS
   mbutil
   m
   opencv_core
   opencv_imgproc
   opencv_highgui
   opencv_video
   opencv_legacy
   pthread)

