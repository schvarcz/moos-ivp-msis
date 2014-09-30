#--------------------------------------------------------
# The CMakeLists.txt for:                       pLocalizationPinger
# Author(s):                              Équipe Ciscrea - Veni Vidi Vici
#--------------------------------------------------------

# FILE(GLOB SRC *.cpp)

INCLUDE_DIRECTORIES( /usr/local/include /usr/local/include/ibex /home/schvarcz/RenataComp/ibex/soplex-1.7.0/src )
SET(SRC
  filtroParticulas.cpp
  hybrid.cpp
  landmark.cpp
  particula.cpp
  robo.cpp
  LocalizationPinger.cpp
  LocalizationPinger_Info.cpp
  main.cpp
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -frounding-math")

ADD_EXECUTABLE(pLocalizationPinger ${SRC})

TARGET_LINK_LIBRARIES(pLocalizationPinger
   MOOS
   mbutil
   m
   ibex
   opencv_core
   opencv_imgproc
   opencv_highgui
   opencv_video
   opencv_legacy
   pthread)

