set(srcs "main.c")

set(requires "bluepad32")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino")
endif()

#commented it out so it works with vscode
# idf_component_register(SRCS "PIDACTUALLYWORKS.cpp" "PID3.cpp" "motor_controller.cpp" "QTR_Controller.cpp" "${srcs}"
#                    INCLUDE_DIRS "."
#                    REQUIRES "${requires}")
#) 

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")

