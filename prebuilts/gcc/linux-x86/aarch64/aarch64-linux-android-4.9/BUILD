This prebuilt was generated using Android's GCC source:
https://android.googlesource.com/toolchain/gcc/

Specifically, the steps for building are:
1. sudo apt-get install -y texinfo gcc-mingw-w64 bison flex libtool
2. mkdir android_gcc; cd android_gcc
3. repo init -u https://android.googlesource.com/platform/manifest -b gcc
4. repo sync
5. cd toolchain/gcc

6. (for plugin support) comment out this line from "build-gcc.sh":
   "run rm -rf $TOOLCHAIN_INSTALL_PATH/lib/gcc/$ABI_CONFIGURE_TARGET/*/plugin"

7. python build.py --toolchain aarch64-linux-android
8. cd ../../out/dist
9. tar -xf gcc-arm64-linux-x86_64.tar.bz2

10. find . -name bversion.h
   Open the found file and replace BUILDING_GCC_PATCHLEVEL with 0 (instead of "4.9" or such)

11. Copy the untar'd directory to the parent directory of this file

For any additional questions: smenashe@magicleap.com
