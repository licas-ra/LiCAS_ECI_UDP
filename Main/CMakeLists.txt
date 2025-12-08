cmake_minimum_required(VERSION 2.8...3.5)

# Include project directories
include_directories( ${LICAS_ECI_SOURCE_DIR}/LiCAS_ECI_UDP )

# Generate the executable from the Main.cpp source code
add_executable( LiCAS_ECI Main.cpp )

# Link the executable to the libraries. 
target_link_libraries( LiCAS_ECI LiCAS_ECI_UDP -pthread )

