#!/bin/bash
declare -r TRUE=1
declare -r FALSE=0
declare -r PROJECT_DIR=$PWD
declare -r DEPENDENCIES_DIR="$PROJECT_DIR"/deps
declare CORES_NUMBER=`nproc`
declare -r INSTALL_DIR="/usr/local"
declare -r SLEEP_FOR=2

printIFVerbose()
{
    if [ $VERBOSE==$TRUE ];then
        echo "$@"
    fi
}
checkFreeRAM()
{
  freeRAM=`free -g |  awk '{print $NF}' | tr "\n" " " | awk '{print $2}'`
}

if [ $CORES_NUMBER -ge 4 ]; then
    checkFreeRAM
    printIFVerbose "[INFO] AVAILABLE RAM MEMORY: " $freeRAM "GB"
    if [ $freeRAM -le 16 ]; then
        CORES_NUMBER=2
        printIFVerbose "[INFO] Insuficient RAM" 
        printIFVerbose "[INFO] Setting up the number of used cores to: " $CORES_NUMBER

    fi
fi

echo "CPU Cores :" $CORES_NUMBER

splitString(){

    local string=$(echo "$1")    
    local delimiter="$2"
    
    # check if the delimiter is null
    # if yes, use " " as delimiter
    if [ -z $delimiter ]; 
    then 
        splitted_string=($(echo "$string" | tr " " "\n"))
    else
        splitted_string=($(echo "$string" | tr $delimiter "\n"))
    fi
}

containsElement () {
  local e match="$1"
  shift
  for e; 
    do 
        [[ "$e" == "$match" ]] && return 0; 
    done
  return 1
}
getUbuntuVersion()
{
    local distro_name=$(lsb_release -d)

    local delimiter=' '
    splitString  "$distro_name"   "$delimiter"

    re='^[0-9]*$' # to check if is a number

    for i in "${splitted_string[@]}"
    do
        if [ -z "${i//[0-9]*/}" -a ! -z "$i" ]; then #is a number
            UBUNTU_MAJOR_VERSION="${i:0:2}" 
            UBUNTU_VERSION="$i"
            return $MAJOR_VERSION
        fi
    done
    echo "vish"
    return 0
}
checkDistro()
{
    local target_distro="$1"
    local test=$target_distro
    # first get the complete distro name and split it into a string
    local distro_name=$(lsb_release -d)
	
    printIFVerbose "[INFO] Distro Name: " $distro_name

    local delimiter=' '
    splitString  "$distro_name"   "$delimiter"

    containsElement "$target_distro" "$splitted_string[@]"

    if [ $?  -eq 1 ];
    then 
        printIFVerbose "[INFO] DISTRO NAME: $target_distro"
        TARGET_DISTRO=$target_distro
        return $TRUE
    else
        return $FALSE
    fi
}

checkVerboseFromUserInput()
{
    VERBOSE=$FALSE
    BUILD_OPENCV=$FALSE
    BUILD_G2O=$FALSE
    BUILD_ARUCO=$FALSE
    INSTALL_DEPENDENCIES=$FALSE
    BUILD_PCL=$FALSE
    BUILD_RGBD_RTK=$TRUE
    BUILD_EIGEN=$FALSE
    options=" "
    for var in "$@"
    do
        if [ $var == '-a' ];
        then
            BUILD_OPENCV=$TRUE
            BUILD_G2O=$TRUE
            BUILD_ARUCO=$TRUE
            INSTALL_DEPENDENCIES=$TRUE
            BUILD_PCL=$TRUE
            BUILD_RGBD_RTK=$TRUE
            BUILD_EIGEN=$TRUE
        fi
        if [ $var == "-v" ];
        then
            VERBOSE=$TRUE
            options="${options} -v"
        elif [ $var == "-install_dependencies" ];
        then
            INSTALL_DEPENDENCIES=$TRUE
            options="${options} -install_dependencies"
        elif [ $var == "-BUILD_OPENCV" ];
        then
             BUILD_OPENCV=$TRUE
             options="${options} -BUILD_OPENCV"
        elif [ $var == "-BUILD_G2O" ];
        then
            BUILD_G2O=$TRUE
            options="${options} -BUILD_G2O"
        elif [ $var == "-BUILD_ARUCO" ];
        then
            BUILD_ARUCO=$TRUE
            options="${options} -BUILD_ARUCO"
        elif [ $var == "-BUILD_PCL" ];
        then
            BUILD_PCL=$TRUE
            options="${options} -BUILD_PCL"
        elif [ $var == "-BUILD_EIGEN" ];
	then
            BUILD_EIGEN=$TRUE
            options="${options} -BUILD_EIGEN"
        fi
    done
    if [ $VERBOSE -eq $FALSE ];then
        echo "VERBOSE MODE OFF"
    else
        printIFVerbose "[INFO] VERBOSE MODE ON"
        printIFVerbose "[INFO] ENABLED OPTIONS: $options"
    fi
    

}

updateGitSubmodules()
{
    printIFVerbose "[INFO] updating git submobules"
    sleep $SLEEP_FOR
    cd $PROJECT_DIR
    git submodule update --init --recursive	
}
buildEigen()
{
    clear
    printIFVerbose "[INFO] building Eigen"
    sleep $SLEEP_FOR
    cd $DEPENDENCIES_DIR/eigen
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR  -DCXX_STANDARD=17 ..
    make install
}
buildOpenCV()
{
    clear
    printIFVerbose "[INFO] building OpenCV"
    sleep $SLEEP_FOR
    cd $DEPENDENCIES_DIR/opencv
    mkdir build
    cd build
    cmake  -DOPENCV_EXTRA_MODULES_PATH=$DEPENDENCIES_DIR/opencv-contrib/modules -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR  -DCMAKE_CXX_FLAGS="--std=c++17" \
    -DWITH_FREETYPE=ON -DOPENCV_ENABLE_NONFREE=ON  Eigen3_DIR=$INSTALL_DIR/share/eigen3/cmake -DWITH_GTK_2_X=ON ..
    make -j$CORES_NUMBER
    make install
    cd $PROJECT_DIR
}

buildG2O()
{
    clear
    printIFVerbose "[INFO] building G2O"
    sleep $SLEEP_FOR
    cd $DEPENDENCIES_DIR/g2o
    mkdir build 
    cd build
    cmake -DG2O_USE_CSPARSE=ON  -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_CXX_FLAGS="--std=c++17" ..
    make -j$CORES_NUMBER
    
    make install
    cd $PROJECT_DIR
}

buildAruco()
{
    clear
    printIFVerbose "[INFO] building Aruco"
    sleep $SLEEP_FOR
    cd $DEPENDENCIES_DIR
    local filename=aruco-3.1.12.zip
    local aruco_dir=aruco-3.1.12
    echo COMPILING ARUCO...
    if [ ! -d $aruco_dir ]; then
        wget hwget https://sourceforge.net/projects/aruco/files/$filename
        unzip $filename
        rm $filename
    fi
    cd $aruco_dir
    mkdir build
    cd build
    # this not work yet, the findAruco.cmake file is not compatible
    #      cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DOpenCV_DIR=$INSTALL_DIR/lib/cmake/opencv4 ..
    cmake  -DOpenCV_DIR=$INSTALL_DIR/lib/cmake/opencv4  ..
    make -j$CORES_NUMBER
    make install
    cd $PROJECT_DIR

    
}

buildPCL()
{
    clear
    printIFVerbose "[INFO] building PCL"
    sleep $SLEEP_FOR
    cd $DEPENDENCIES_DIR/pcl
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR  -DCMAKE_CXX_STANDARD=17 ..
    make -j$CORES_NUMBER && make install
    cd $PROJECT_DIR
}


installUbuntuDependencies()
{   
    clear
    printIFVerbose "[INFO] Installing ubuntu dependencies"
    sleep $SLEEP_FOR
    apt-get update || $(echo "apt-get update failed" exit  )
    apt install -y build-essential || echo "apt install build-essential failed" exit
    apt install -y cmake || echo "apt install cmake failed" exit
    apt install -y libusb-1.0-0-dev || echo "apt install libusb-1.0-0-dev failed" exit
    apt install -y libsuitesparse-dev || echo "apt install libsuitesparse-dev failed" exit
    apt install -y libboost-all-dev || echo "apt install libboost-all-dev failed" exit
    apt install -y libvtk7-dev | echo "apt install libvtk7-dev failed" exit
    apt install -y libflann-dev | echo "apt install libflann-dev failed" exit
}
buildRGBD_RTK()
{
    printIFVerbose "[INFO] building the project RGBD-RTK"
    sleep $SLEEP_FOR
    cd $PROJECT_DIR
    mkdir build
    cd build
    echo $INSTALL_DIR/lib/cmake/g2o
    cmake -DOpenCV_DIR=$INSTALL_DIR/lib/cmake/opencv4 -DG2O_DIR=$INSTALL_DIR ..
    make
} 
installGCC9()
{
    	printIFVerbose "[INFO] Installing GCC and G++ version 9.4"
    	sleep $SLEEP_FOR
	add-apt-repository ppa:ubuntu-toolchain-r/test
	apt-get update
	apt install gcc-9
	apt install g++-9
	unlink /usr/bin/g++
	unlink /usr/bin/gcc
	ln -sn /usr/bin/g++-9 /usr/bin/g++
	ln -sn /usr/bin/gcc-9 /usr/bin/gcc
}


############# MAIN CODE #############
main()
{ 
    if [ `whoami` != "root" ];
    then
        printIFVerbose "[ERROR] YOU NEED ROOT PERMISSION TO RUN THIS SCRIPT"
        exit 1

    fi

    if [ ! -d $DEPENDENCIES_DIR ]; 
    then
    	mkdir $DEPENDENCIES_DIR
    fi
    kernel_name=$(uname -s)

    checkVerboseFromUserInput "$@"



    printIFVerbose "[INFO] Kernel Name: $kernel_name"

    if [ $kernel_name == "Linux" ];
    then    
        updateGitSubmodules

        checkDistro "Ubuntu"
        is_valid_distro=$?

        if [ $is_valid_distro -eq $TRUE ];
        then
            if [ $TARGET_DISTRO == "Ubuntu" ];
            then 
                getUbuntuVersion
                
                if [ $UBUNTU_MAJOR_VERSION -ge 18 -a $INSTALL_DEPENDENCIES -eq $TRUE ]; then
                    installUbuntuDependencies
                elif [ $INSTALL_DEPENDENCIES -eq $TRUE ]; then
                	installGCC9
                	installUbuntuDependencies
                	echo "G++ and GCC upgraded"
                	echo "missing:"
                	echo "libboost (1.7+)"
                	echo "cmake (3.16+)"
                	
                fi
               
            else
                echo "your Distro is not compatible"
                exit
            fi

            if [ $BUILD_EIGEN -eq $TRUE ]; then
                buildEigen
            fi

	        if [ $BUILD_OPENCV -eq $TRUE ]; then
                buildOpenCV
            fi
            
            if [ $BUILD_G2O -eq $TRUE ]; then
                buildG2O
            fi
            
            if [ $BUILD_ARUCO -eq $TRUE ]; then
                buildAruco
            fi
            if [ $BUILD_PCL -eq $TRUE ]; then
            	buildPCL
            fi
            
            if [ $BUILD_RGBD_RTK -eq $TRUE ]; then
               buildRGBD_RTK
	        fi
        elif [ $is_valid_distro -eq $FALSE ];
        then
            echo "Invalid Distro"
        fi
    else   
        echo "ERROR: the system only support linux OS"
        exit 1
    fi

}

main "$@"
