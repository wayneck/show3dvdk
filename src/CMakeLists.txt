set(TARGET_NAME trainpilot-client)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

file(GLOB_RECURSE PROTO_SOURCE_FILES ${PROTO_DIR}/*.cc)
file(GLOB_RECURSE PROTO_HEADER_FILES ${PROTO_DIR}/*.h)

set(RESOURCE_FILE ./resource.qrc)

add_executable (${TARGET_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${PROTO_SOURCE_FILES}
    ${PROTO_HEADER_FILES}
    ${RESOURCE_FILE}
)

target_link_libraries (${TARGET_NAME} 
    Qt5::Widgets Qt5::Gui Qt5::Core Qt5::OpenGL Qt5::Network
	${Boost_LIBRARIES}
	${PROTOBUF_LIBRARIES}
	${OpenCV_LIBRARIES}
	${VTK_LIBRARIES}
	${PCL_LIBRARIES}    
    pthread
    rt
    )
