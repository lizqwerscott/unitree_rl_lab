include(ExternalProject)

# 全局策略和编译参数
set(COMMON_CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${LOCAL_INSTALL_DIR}
                      -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17)

set(common_project_args UPDATE_DISCONNECTED ON BUILD_ALWAYS FALSE)

set(external_project_args)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  list(APPEND external_project_args DOWNLOAD_EXTRACT_TIMESTAMP YES)
endif()

# --------------------- 依赖定义 ---------------------
set(BOOST_VERSION 1.88.0)

include(ExternalProject)
ExternalProject_Add(
  boost
  URL "https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-cmake.tar.gz"
  CONFIGURE_COMMAND
    ./bootstrap.sh --with-libraries=filesystem,system,serialization,test,program_options
    --prefix=${LOCAL_INSTALL_DIR}
  BUILD_COMMAND ./b2 link=static,shared threading=multi runtime-link=shared
                --prefix=${LOCAL_INSTALL_DIR} install
  BUILD_IN_SOURCE TRUE
  INSTALL_COMMAND "" # 禁用默认安装步骤
  ${common_project_args} ${external_project_args})

ExternalProject_Add(
  eigen
  GIT_REPOSITORY https://github.com/eigen-mirror/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
  BUILD_COMMAND "" # Eigen是纯头文件库，无需编译
  INSTALL_COMMAND cmake --build . --target install
  ${common_project_args})

ExternalProject_Add(
  tinyxml2
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2
  GIT_TAG 11.0.0
  GIT_SHALLOW TRUE
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
             # -DCMAKE_POLICY_VERSION_MINIMUM=3.5
             -DCMAKE_CXX_FLAGS=-fPIC # 显式传递编译标志
             -DCMAKE_C_FLAGS=-fPIC
  ${common_project_args})

ExternalProject_Add(
  console_bridge_project
  GIT_REPOSITORY https://github.com/ros/console_bridge.git
  GIT_TAG 1.0.2
  GIT_SHALLOW TRUE
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
             -DCMAKE_POLICY_VERSION_MINIMUM=3.5
             -DCMAKE_CXX_FLAGS=-fPIC # 显式传递编译标志
             -DCMAKE_C_FLAGS=-fPIC
             -DBUILD_SHARED_LIBS=FALSE
  ${common_project_args})

ExternalProject_Add(
  urdfdom_headers
  GIT_REPOSITORY https://github.com/ros/urdfdom_headers.git
  GIT_TAG 1.1.2
  GIT_SHALLOW TRUE
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
  ${common_project_args})

ExternalProject_Add(
  urdfdom
  GIT_REPOSITORY https://github.com/ros/urdfdom.git
  # GIT_TAG 4.0.1
  GIT_SHALLOW TRUE
  DEPENDS urdfdom_headers tinyxml2 console_bridge_project
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
  ${common_project_args})

# ExternalProject_Add(assimp GIT_REPOSITORY https://github.com/assimp/assimp.git
# CMAKE_ARGS        ${COMMON_CMAKE_ARGS} -DBUILD_SHARED_LIBS=OFF )

# ExternalProject_Add(hpp-fcl GIT_REPOSITORY
# https://github.com/humanoid-path-planner/hpp-fcl.git DEPENDS           assimp
# urdfdom CMAKE_ARGS        ${COMMON_CMAKE_ARGS}
# -DCMAKE_PREFIX_PATH=${LOCAL_INSTALL_DIR} -DBUILD_PYTHON_INTERFACE=OFF )

set(PINOCCHIO_VERSION 3.6.0)
ExternalProject_Add(
  pinocchio
  URL "https://github.com/stack-of-tasks/pinocchio/releases/download/v${PINOCCHIO_VERSION}/pinocchio-${PINOCCHIO_VERSION}.tar.gz"
  DEPENDS eigen boost urdfdom
  CMAKE_ARGS ${COMMON_CMAKE_ARGS}
             # -DBUILD_WITH_COLLISION_SUPPORT=ON
             -DBUILD_PYTHON_INTERFACE=OFF
             -DBUILD_TESTING=OFF
  ${common_project_args} ${external_project_args})

