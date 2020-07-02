set(TARGET_NAME show_vdk)

MESSAGE("GL" ${OPENGL_LIBRARIES})
MESSAGE("------------------")
MESSAGE("GL" ${GLUT_LIBRARY})
add_executable(${TARGET_NAME}  PLYLoader.cpp displayvlp.cpp)
target_link_libraries(${TARGET_NAME}  
	${OpenCV_LIBRARIES}
	${VTK_LIBRARIES}
	${PCL_LIBRARIES}   
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    )

add_executable(getangle  getangle.cpp)
target_link_libraries(getangle  
	${OpenCV_LIBRARIES}
	${VTK_LIBRARIES}
	${PCL_LIBRARIES}   
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    )

add_executable(chull_pcd  chull_pcd.cpp)
target_link_libraries(chull_pcd  
	${OpenCV_LIBRARIES}
	${VTK_LIBRARIES}
	${PCL_LIBRARIES}   
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    )

add_executable(mls_pcd  mls_pcd.cpp)
target_link_libraries(mls_pcd  
	${OpenCV_LIBRARIES}
	${VTK_LIBRARIES}
	${PCL_LIBRARIES}   
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY} 
    )
