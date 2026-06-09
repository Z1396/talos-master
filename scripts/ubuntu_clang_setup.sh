#!/bin/bash

apt install wget curl unzip zip ninja-build make git

wget https://apt.llvm.org/llvm.sh
chmod +x llvm.sh
./llvm.sh 21 all -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt

apt install -y libgtest-dev nlohmann-json3-dev libceres-dev libusb-1.0-0-dev

# update default linker, compiler to llvm-21
update-alternatives --install /usr/bin/clang clang /usr/bin/clang-21 100
update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-21 100
update-alternatives --install /usr/bin/ld ld /usr/bin/lld-21 100

# clangd, clang-format, clang-tidy, etc...
# update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-21 100
update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-21 100
update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-21 100
update-alternatives --install /usr/bin/clang-query clang-query /usr/bin/clang-query-21 100
update-alternatives --install /usr/bin/clang-include-fixer clang-include-fixer /usr/bin/clang-include-fixer-21 100

add-apt-repository ppa:ubuntu-toolchain-r/test
# replace /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.list ppa.launchpadcontent.net to launchpad.proxy.ustclug.org

for f in /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.list; do
    cp $f $f.bak
    sed -i 's/ppa.launchpadcontent.net/launchpad.proxy.ustclug.org/g' $f
done

for f in /etc/apt/sources.list.d/ubuntu-toolchain-r-ubuntu-test-*.sources; do
    cp $f $f.bak
    sed -i 's/ppa.launchpadcontent.net/launchpad.proxy.ustclug.org/g' $f
done

apt update
apt install -y g++-13 gcc-13
apt install libgcc-13-dev libstdc++-13-dev
