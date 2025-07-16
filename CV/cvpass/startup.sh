## 
# the compile script for loongson car
# use make to compile the out file
# use clean to clear last built
##

#make
if test $1 = "make"
then
    rm -rf ./build
    cmake -B build
    cmake --build build
    cp ./build/main ./cv
    sshpass -p '123' scp ./cv loongson@192.168.3.12:/home/loongson
fi

#clean
if test $1 = "clean"
then
    rm -rf ./build
fi