# OpenGL
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

# GUI
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

add_library(MyGL
	Window.h
	Window.cpp
	Shader.h
	Shader.cpp
	Camera.h
	Camera.cpp
	Mesh.h
	Mesh.cpp
	LineSegment.h
	LineSegment.cpp
	PointCloud.h
	PointCloud.cpp
	PickVertex.h
	PickVertex.cpp
	LogConsole.h
	LogConsole.cpp
)

target_include_directories(MyGL PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MyGL PUBLIC
	glad::glad
	glfw
	imgui::imgui
)