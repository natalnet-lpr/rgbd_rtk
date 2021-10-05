#!/bin/bash
declare -r TRUE=1
declare -r FALSE=0


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

    if [ $VERBOSE -eq $TRUE ]; then
        echo "Distro Name: " $distro_name
    fi

    local delimiter=' '
    splitString  "$distro_name"   "$delimiter"

    containsElement "$target_distro" "$splitted_string[@]"

    if [ $?  -eq 1 ];
    then 
        if [ $VERBOSE -eq $TRUE ];
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
    BUILD_G2O=$FALSE
    BUILD_ARUCO=$FALSE
    INSTALL_DEPENDENCIES=$FALSE
    options=" "
    for var in "$@"
    do
        if [ $var == "-v" ];
        then
            VERBOSE=$TRUE
            options="${options} -v"
        elif [ $var == "-install_dependencies" ];
        then
            INSTALL_DEPENDENCIES=$TRUE
            options="${options} -install_dependencies"
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

buildG2O()
{
    echo COMPILING G2O...
    wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20201223_git.tar.gz
    tar -xf 20201223_git.tar.gz
    rm 20201223_git.tar.gz
    cd g2o-20201223_git
    mkdir build
    mkdir install
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=../install -DG2O_USE_CSPARSE=ON ..
    make -j4
    make install
}

buildAruco()
{
    local filename=aruco-3.1.12.zip
    local aruco_dir=aruco-3.1.12
    echo COMPILING ARUCO...
    wget hwget https://sourceforge.net/projects/aruco/files/$filename
    unzip $filename
    rm $filename
    cd $aruco-3.1.12
    mkdir build
    cd build
    cmake ..
    make && sudo make install

    
}

installUbuntuDependencies()
{   
    echo INSTAL install_dependencies?  $INSTALL_DEPENDENCIES $TRUE
    sudo apt-get update || $(echo "apt-get update failed" exit  )
    sudo apt install build-essential || echo "apt install build-essential failed" exit
    sudo apt install libopencv-dev || echo "apt install opencv failed"  exit
    sudo apt install cmake || echo "apt install cmake failed" exit
    sudo apt install libpcl-dev || echo "apt install libpcl-dev failed"  exit
    sudo apt install libusb-1.0-0-dev || echo "apt install libusb-1.0-0-dev failed" exit
    sudo apt install libsuitesparse-dev || echo "apt install libsuitesparse-dev failed" exit
} 


############# MAIN CODE #############
main()
{

    kernel_name=$(uname -s)

    checkVerboseFromUserInput "$@"


    if [ $VERBOSE -eq $TRUE ];
    then
        echo "Kernel Name: $kernel_name"
    fi

    if [ $kernel_name == "Linux" ];
    then    
        checkDistro "Ubuntu"
        is_valid_distro=$?

        if [ $is_valid_distro -eq $TRUE ];
        then
            if [ $TARGET_DISTRO == "Ubuntu" ];
            then 
                getUbuntuVersion
                
                if [ $UBUNTU_MAJOR_VERSION -ge 20 -a $INSTALL_DEPENDENCIES -eq $TRUE ]; then
                    installUbuntuDependencies
                fi
            else
                echo "your Ubuntu Version ($UBUNTU_VERSION) is not compatible"
                exit
            fi

            if [ $BUILD_G2O -eq $TRUE ]; then
                buildG2O
            fi
            if [ $BUILD_ARUCO -eq $TRUE ]; then
                echo UHUU
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