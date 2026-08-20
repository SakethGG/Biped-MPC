add_executable(main
	main.cpp
	main_helper.cpp
)

target_link_libraries(main PRIVATE
	sim
	My_Controller
)

add_executable(command_com_trajectory
	command_com_trajectory.cpp
	main_helper.cpp
)

target_link_libraries(command_com_trajectory PRIVATE
	sim
	My_Controller
)
