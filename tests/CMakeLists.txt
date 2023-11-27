set(Tests 
	dummy.cpp
	world_normal.cpp
	# mmap_buffer.cpp
	# fs_chunk.cpp
	# utm_coordinates.cpp
)

include_directories(../include)

# Add all the ADD_TEST for each test
foreach (test ${Tests})
	get_filename_component (TName ${test} NAME_WE)
	add_executable(${TName} ${test} ${CMAKE_CURRENT_SOURCE_DIR})
	target_link_libraries(${TName} ld54)
	target_link_libraries(${TName} g)
	target_link_libraries(${TName} glfw)
	target_link_libraries(${TName} ${OPENAL_LIBRARY})
	add_test(NAME ${TName} COMMAND ${TName})
endforeach ()
