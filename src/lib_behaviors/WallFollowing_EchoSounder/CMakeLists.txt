#--------------------------------------------------------
# The CMakeLists.txt for:       WallFollowing_EchoSounder
# Author(s):                                  Simon Rohou
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})


MACRO(ADD_BHV BHV_NAME)
  ADD_LIBRARY(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${BHV_NAME} 
    helmivp
    behaviors 
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    mbutil 
    geometry
    ${SYSTEM_LIBS} )
ENDMACRO(ADD_BHV)


#--------------------------------------------------------
#                                WallFollowing_EchoSounder
#--------------------------------------------------------
ADD_LIBRARY(BHV_WallFollowing_EchoSounder SHARED 
   WallFollowing_EchoSounder.cpp)
TARGET_LINK_LIBRARIES(BHV_WallFollowing_EchoSounder
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   geometry
   ${SYSTEM_LIBS} )
