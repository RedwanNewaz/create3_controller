set(FIND_SENSOR_FUSION_PATHS
        /home/airlab/CLionProjects/sensor-fusion
)

set(SENSOR_FUSION_INCLUDE_DIRS
        /home/airlab/CLionProjects/sensor-fusion/include
)

find_library(SENSOR_FUSION_LIBRARIES
    NAMES sensor_fusion
    PATH_SUFFIXES build
    PATHS ${FIND_SENSOR_FUSION_PATHS}
)