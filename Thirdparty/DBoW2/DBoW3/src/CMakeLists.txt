INCLUDE_DIRECTORIES(. ./utils/)

FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.c*")

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )


ADD_LIBRARY(${PROJECT_NAME} ${srcs} ${hdrs})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ) 

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL DBOW_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
	OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

INSTALL(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin COMPONENT main			# Install the dll file in bin directory
	LIBRARY DESTINATION ${LIB_INSTALL_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE COMPONENT main
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT main)			# Install the dll.a file in lib directory



INSTALL(FILES ${hdrs_base}
        DESTINATION include/${PROJECT_NAME}
        COMPONENT main)



