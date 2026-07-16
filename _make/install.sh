#!/bin/bash

lib=${@: -1}
init_path=$PWD
usage() { echo "Usage: $0 [-gtc] [-v <version>] <package>" 1>&2; }

### variable defining dependency packages

ubuntu_rai="g++ clang make gnupg cmake git wget libstdc++-14-dev \
        	liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev \
		libjsoncpp-dev libyaml-dev libhdf5-dev \
        	libx11-dev libxi-dev libxxf86vm-dev libglu1-mesa-dev \
		libglfw3-dev libglew-dev libglm-dev freeglut3-dev \
		libfreetype-dev fonts-ubuntu \
		libpng-dev libassimp-dev"

ubuntu_python="python3-dev python3 python3-pip python3-venv"

ubuntu_botop="libpoco-dev libboost-system-dev \
		portaudio19-dev libusb-1.0-0-dev libhidapi-dev"

ubuntu_robotic="liblapack3 freeglut3 libglu1-mesa libxrandr2 \
		libfreetype6 fonts-ubuntu \
		python3 python3-pip"

if [ $lib = "vars_only" ]; then
    return 0
fi


### additional options (global tmp clean version)

git=${HOME}/git
pre=${HOME}/.local

while getopts "gtcv:" o; do
    case "${o}" in
        g)
            pre=/usr/local
            ;;
        t)
	    git=/tmp/git
	    pre=/tmp/local
            ;;
	c)
	    clean="yes"
	    ;;

	v)
	    version=${OPTARG}
            ;;
	
        *)
	    echo '=== option' ${o} 'unknown - ignored'
            usage
            ;;
    esac
done

mkdir -p ${git}
mkdir -p ${pre}


### install

echo '=== Installing' ${lib} ' -- sources:' ${git} ' -- prefix (compiled library):' ${pre} ' -- version' ${version} ' -- clean' ${clean}

cd ${git}

case ${lib} in

    botop)
	git clone --recurse-submodules https://github.com/MarcToussaint/botop.git
	export PY_VERSION=`python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))"`
	cmake -DPY_VERSION=$PY_VERSION -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    rai)
	git clone --single-branch -b marc https://github.com/MarcToussaint/rai.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DUSE_PHYSX=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    libfranka)
	if [ -z "$version" ]; then version="0.10.0"; fi #old: 0.8.0 very old: 0.7.1
	git clone --single-branch -b ${version} --recurse-submodules https://github.com/frankaemika/libfranka
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    libfranka-static)
	if [ -z "$version" ]; then version="0.10.0"; fi #old: 0.7.1
	git clone --single-branch -b ${version} --recurse-submodules https://github.com/frankaemika/libfranka
	cd libfranka
	wget https://github.com/MarcToussaint/rai-extern/raw/main/franka.patch
	patch -p1 CMakeLists.txt franka.patch
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF . -B build
	make -C build install
	;;

    physx)
	if [ -z "$version" ]; then version="105.1-physx-5.3.1"; fi # release/104.2 106.1-physx-5.4.2
        git clone --single-branch -b ${version} https://github.com/NVIDIA-Omniverse/PhysX.git
	cd PhysX/physx/buildtools/packman; chmod a+x packman; ./packman update -y
        cd ../..; ./generate_projects.sh linux; #linux-gcc-cpu-only.xml;
	cd compiler/linux-release; cmake . -DPX_BUILDPVDRUNTIME=OFF -DPX_BUILDSNIPPETS=OFF -DCMAKE_INSTALL_PREFIX=${pre} -DCMAKE_CXX_FLAGS='-w'
	make install
	cd ${pre}/lib; ln -sf ../bin/linux.clang/release PhysX
	;;
    
    librealsense)
	#sudo apt install --yes libusb-1.0-0-dev libglfw3-dev libgtk-3-dev
        git clone --single-branch -b v2.58.1 --recurse-submodules https://github.com/IntelRealSense/librealsense.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DCMAKE_BUILD_TYPE=Release -DBUILD_ROSBAG2=OFF -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_GLSL_EXTENSIONS=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    eigen)
	git clone --single-branch -b 3.4.0 https://gitlab.com/libeigen/eigen.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    jsoncpp)
	git clone --single-branch -b 1.9.5 https://github.com/open-source-parsers/jsoncpp.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    libyaml)
	git clone --single-branch -b 0.2.5 https://github.com/yaml/libyaml.git
	cd ${lib}; ./bootstrap
	./configure --prefix=${pre} CFLAGS='-fPIC'
	make install
	;;

    libpng)
	git clone --single-branch -b libpng16 https://github.com/glennrp/libpng.git
	cd ${lib}; ./configure --prefix=${pre} CFLAGS='-fPIC'
	make install
	;;

    hdf5)
	git clone --single-branch -b hdf5-1_10_4 https://github.com/HDFGroup/hdf5.git
	cd ${lib}; env CFLAGS="-fPIC" CXXFLAGS="-fPIC" ./configure --prefix=${pre} --enable-cxx --disable-tests --disable-tools --disable-shared
	make install
	;;

    qhull)
	git clone --single-branch -b v7.3.2 https://github.com/qhull/qhull.git
	env CFLAGS="-fPIC" CXXFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build2
	make -C ${lib}/build2 install
	cd ${pre}/lib; ln -sf libqhullstatic.a libqhull.a; cd ${pre}/include; ln -sf libqhull qhull
	;;

    libccd)
	git clone --single-branch -b v2.1 https://github.com/danfis/libccd.git
	env CFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_SHARED_LIBS=ON ${lib} -DENABLE_DOUBLE_PRECISION=ON -B ${lib}/build
	make -C ${lib}/build install
	env CFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_SHARED_LIBS=OFF ${lib} -DENABLE_DOUBLE_PRECISION=ON -B ${lib}/build  #compile shared AND static versions!
	make -C ${lib}/build install
	;;

    fcl)
	git clone --single-branch -b fcl-0.5 https://github.com/flexible-collision-library/fcl.git
        cmake -DCMAKE_INSTALL_PREFIX=${pre} -DFCL_STATIC_LIBRARY=ON -DFCL_BUILD_TESTS=OFF -DFCL_WITH_OCTOMAP=OFF -DCMAKE_CXX_FLAGS="-Wno-deprecated-copy -Wno-class-memaccess -Wno-maybe-uninitialized" ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    coal)
	# libboost-serialization-dev libboost-filesystem-dev
	git clone --single-branch -b devel https://github.com/coal-library/coal.git
        cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF -DAUTO_UNINSTALL=OFF -DBUILDING_ROS2_PACKAGE=OFF -DBUILD_DOCUMENTATION=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;
    
    glew)
	wget https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz; tar xvzf glew-2.2.0.tgz
	env GLEW_DEST=${pre} make -C glew-2.2.0 install
	;;

    freeglut)
	git clone --single-branch -b v3.6.0 https://github.com/freeglut/freeglut.git
	env CFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    glfw)
	git clone --single-branch -b 3.3-stable https://github.com/glfw/glfw.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    assimp)
	git clone --single-branch -b v5.2.5 https://github.com/assimp/assimp.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DASSIMP_BUILD_TESTS=OFF -DASSIMP_BUILD_ZLIB=ON -DBUILD_SHARED_LIBS=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    bullet)
	git clone --single-branch -b 3.08 https://github.com/bulletphysics/bullet3.git bullet
	env CFLAGS="-fPIC" CXXFLAGS="-fPIC" cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_SHARED_LIBS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	;;

    libann)
	git clone --single-branch https://github.com/daveb-dev/libANN.git libann
	make -C ${lib}/src targets "ANNLIB=libANN.a" "C++=g++" "CFLAGS=-O3 -fPIC" "MAKELIB=ar ruv" "RANLIB=true"
	cp -v ${lib}/lib/libANN.a ${pre}/lib; cp -vR ${lib}/include/ANN ${pre}/include
	;;

    opencv)
	git clone --single-branch -b 5.0.0 https://github.com/opencv/opencv.git
	#git clone --single-branch -b 4.7.0 https://github.com/opencv/opencv_contrib.git
	cmake -DCMAKE_INSTALL_PREFIX=${pre} -DBUILD_opencv_dnn=OFF -DBUILD_opencv_python3=OFF -DWITH_VTK=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF ${lib} -B ${lib}/build
	make -C ${lib}/build install
	#find ${lib}/build/3rdparty -name 'lib*.a' -exec cp {} ${pre}/lib \;
	;;

    basler)
	cd ${pre}
	scp hal-9000.lis.tu-berlin.de:/home/data/pylon-26-06-minimal.tgz .
	tar xvzf pylon-26-06-minimal.tgz
	#tar cvzf pylon-26-06-minimal.tgz include/ lib/libpylonbase.so* lib/libpylonutility* lib/libMathParser_gcc_v3_5_Basler_pylon_v1.so lib/libGenApi_gcc_v3_5_Basler_pylon_v1.so lib/libGCBase_gcc_v3_5_Basler_pylon_v1.so lib/libXmlParser_gcc_v3_5_Basler_pylon_v1.so lib/libLog_gcc_v3_5_Basler_pylon_v1.so lib/libNodeMapData_gcc_v3_5_Basler_pylon_v1.so share/pylon/setup-usb.sh
	;;
	
    test)
	if [ -z "$version" ]; then version="default"; fi
	echo 'just testing -- version:' ${version}
	;;

    isaacsim)
	wget https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone%404.5.0-rc.36%2Brelease.19112.f59b3005.gl.linux-x86_64.release.zip
	cd ${pre}
	mkdir -p isaac-sim
	cd isaac-sim
	unzip
	;;

    vars_only)
	;;
    
    *)
	echo '=== package' ${lib} 'not defined'
	usage
esac


### clean

if [ "${clean}" = "yes" -a -e "${lib}/build" ]; then
    cd ${git}
    echo 'Cleaning' ${lib}/build
    rm -Rf ${lib}/build
fi

cd ${init_path}
