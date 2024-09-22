# 安装所需依赖

```
cd ~
mkdir 3rdparty
cd 3rdparty
git clone https://github.com/ompl/ompl.git
cd ompl
mkdir -p build/Realease
cd build/Realease
cmake ../..
make
sudo make install
```

# requirement

```
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
clang-tidy clang-format python3-matplotlib libompl-dev libeigen3-dev
```

