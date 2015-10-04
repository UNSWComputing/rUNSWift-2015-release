SET(ROBOT_SRCS
   # Vision
   perception/vision/NaoCamera.cpp
   perception/vision/NaoCameraV4.cpp

   # Perception
   perception/PerceptionThread.cpp #nothing robot specific, but we depend on visionadapter

   # Motion
   motion/MotionAdapter.cpp
   motion/effector/AgentEffector.cpp
   motion/touch/AgentTouch.cpp
   )

set_source_files_properties(
   perception/PerceptionThread.cpp
   main.cpp
   PROPERTIES COMPILE_FLAGS "-I${PYTHON_INCLUDE_DIR}")

ADD_LIBRARY(robot-static STATIC ${ROBOT_SRCS} )
TARGET_LINK_LIBRARIES( robot-static soccer-static )
SET_TARGET_PROPERTIES(robot-static PROPERTIES OUTPUT_NAME "robot")
SET_TARGET_PROPERTIES(robot-static PROPERTIES PREFIX "lib")
SET_TARGET_PROPERTIES(robot-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

ADD_EXECUTABLE( runswift main.cpp version.cpp )
TARGET_LINK_LIBRARIES(
   runswift
   robot-static
   ${PTHREAD_LIBRARIES}
   ${RUNSWIFT_BOOST}
   ${PYTHON_LIBRARY}
   # TODO: Not quick hack to make it work.
   # TODO: ... somewhere upstream we are NOT finding BZip2 correctly,
   # TODO: ... i.e. we should be able to just
   # TODO: ... TARGET_LINK_LIBRARIES (runswift ${BZIP2_LIBRARIES})
   # TODO: ... http://www.cmake.org/Wiki/CMake:How_To_Find_Libraries#Using_external_libraries
   ${CTC_DIR}/bzip2/lib/libbz2.so
   # TODO: ... Ditto TARGET_LINK_LIBRARIES (runswift ${ZLIB_LIBRARIES}) should work but doesn't hence hack.
   ${CTC_DIR}/zlib/lib/libz.so
)

ADD_CUSTOM_COMMAND ( TARGET runswift POST_BUILD
   COMMAND rm version.cpp
)
