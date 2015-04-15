include_directories(${PROJECT_SOURCE_DIR}/tests/include)

orocos_component(FriExample src/friExample.cpp) 
orocos_component(FriExampleKinematic src/friExampleKinematic.cpp) 
orocos_component(FriExampleTorque src/friExampleTorque.cpp)
orocos_component(FriExampleTorqueRTNet src/friExampleTorqueRTNet.cpp) 
orocos_component(ATIcalibration src/ATIcalibration-component.cpp) 
#orocos_component(ATICalibrationCartesianImpedance src/ATICalibrationCartesianImpedance.cpp)
orocos_component(FriExampleJacobian src/friExampleJacobian.cpp)
orocos_component(FriExampleJacobianRTNet src/friExampleJacobianRTNet.cpp) # ...you may add multiple source files
orocos_component(RTEKukaFRIClient src/demoKukaKinectKarama-rtnetcomponent.cpp src/movingMean.cpp) # ...you may add multiple source files

target_link_libraries(FriExample friExampleAbstract)
target_link_libraries(FriExampleKinematic friExampleAbstract)
target_link_libraries(FriExampleTorque friExampleAbstract)
target_link_libraries(FriExampleTorqueRTNet friRTNetExampleAbstract)
target_link_libraries(ATIcalibration friRTNetExampleAbstract)
#target_link_libraries(ATICalibrationCartesianImpedance friRTNetExampleAbstract)
target_link_libraries(FriExampleJacobian friExampleAbstract)
target_link_libraries(FriExampleJacobianRTNet friRTNetExampleAbstract)
target_link_libraries(RTEKukaFRIClient friRTNetExampleAbstract)


# You may add multiple orocos_component statements.
#
# Additional headers:
#
# Installs in the include/orocos/fri_examples/ directory

orocos_install_headers(friExample.hpp) 
orocos_install_headers(friExampleKinematic.hpp) 
orocos_install_headers(friExampleTorque.hpp)
orocos_install_headers(friExampleTorqueRTNet.hpp)
orocos_install_headers(ATIcalibration-component.hpp)
#orocos_install_headers(ATICalibrationCartesianImpedance.hpp)
orocos_install_headers(friExampleJacobian.hpp) 
orocos_install_headers(friExampleJacobianRTNet.hpp) # ...you may add multiple header files
orocos_install_headers(demoKukaKinectKarama-rtnetcomponent.hpp movingMean.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

