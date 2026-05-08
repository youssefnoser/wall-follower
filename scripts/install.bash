rm -r build
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-toolchain.cmake