#!/bin/bash
declare -r TRUE=1
declare -r FALSE=0
declare -r PROJECT_DIR=$PWD
declare -r DEPENDENCIES_DIR="$PROJECT_DIR"/deps
declare -r CORES_NUMBER=`nproc`
declare -r INSTALL_DIR="~/Libraries/CPP/Installation"
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

    if [ "$VERBOSE" -eq "$TRUE" ]; then
        echo "Distro Name: " $distro_name
    fi

    local delimiter=' '
    splitString  "$distro_name"   "$delimiter"

    containsElement "$target_distro" "$splitted_string[@]"

    if [ $?  -eq 1 ];
    then 
        if [ "$VERBOSE" -eq "$TRUE" ];
            then echo "DISTRO NAME: $target_distro"
            echo $?
        fi
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
    options=" "
    for var in "$@"
    do
        if [ $var == '-a' ];
        then
            VERBOSE=$TRUEa
            BUILD_OPENCV=$TRUE
            BUILD_G2O=$TRUE
            BUILD_ARUCO=$TRUE
            INSTALL_DEPENDENCIES=$TRUE
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
        fi
    done
    if [ $VERBOSE -eq $FALSE ];then
        echo "VERBOSE MODE OFF"
    else
        echo "VERBOSE MODE ON"
        echo "ENABLED OPTIONS: $options"
    fi
    

}
updateGitSubmodules()
{
    cd $PROJECT_DIR
    git submodule update --init --recursive	
}
buildOpenCV()
{
    cd $DEPENDENCIES_DIR/opencv
    mkdir build
    cd build
    cmake  -DOPENCV_EXTRA_MODULES_PATH=$DEPENDENCIES_DIR/opencv-contrib/modules -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR  -DCMAKE_CXX_FLAGS="--std=c++17" ..
    make -j$CORES_NUMBER
    sudo make install
    cd $PROJECT_DIR
}

buildG2O()
{
    cd $DEPENDENCIES_DIR/g2o
    cd build
    cmake -DG2O_USE_CSPARSE=ON  -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_CXX_FLAGS="--std=c++17" ..
    make -j$CORES_NUMBER
    sudo make install
    cd $PROJECT_DIR
}

buildAruco()
{
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
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
    make -j$CORES_NUMBER && sudo make install
    cd $PROJECT_DIR

    
}

buildPCL()
{
    cd $DEPENDENCIES_DIR/pcl
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
    make -j$CORES_NUMBER && sudo make install
    cd $PROJECT_DIR
}


installUbuntuDependencies()
{   
    echo INSTAL install_dependencies?  $INSTALL_DEPENDENCIES $TRUE
    sudo apt-get update || $(echo "apt-get update failed" exit  )
    sudo apt install build-essential || echo "apt install build-essential failed" exit
    sudo apt install cmake || echo "apt install cmake failed" exit
    sudo apt install libpcl-dev || echo "apt install libpcl-dev failed"  exit
    sudo apt install libusb-1.0-0-dev || echo "apt install libusb-1.0-0-dev failed" exit
    sudo apt install libsuitesparse-dev || echo "apt install libsuitesparse-dev failed" exit
    sudo apt install libboost-all-dev || echo "apt install libboost-all-dev failed" exit

} 


############# MAIN CODE #############
main()
{ 
    if [ ! -d $DEPENDENCIES_DIR ]; 
    then
    	mkdir $DEPENDENCIES_DIR
    fi
    kernel_name=$(uname -s)

    checkVerboseFromUserInput "$@"


    if [ "$VERBOSE" -eq "$TRUE" ];
    then
        echo "Kernel Name: $kernel_name"
    fi

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
                fi
            else
                echo "your Ubuntu Version $UBUNTU_VERSION is not compatible"
                exit
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
