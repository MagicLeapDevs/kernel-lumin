#!/bin/bash -ex

set +x
echo "Building USER kernel..."
echo "Building USER kernel..."
echo "Building USER kernel..."
set -x

export T="`pwd`"
export MAKETHREADS="`nproc`"
export CC=${T}/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin
export CC32=${T}/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/
export PHAEDRAOUT="${T}/out/target/product/phaedra"

mkdir -p ${PHAEDRAOUT}/obj/KERNEL/kernel-4.4/generated
mkdir -p ${PHAEDRAOUT}/obj/KERNEL/kernel-4.4
KERNEL_OUT=${PHAEDRAOUT}/obj/KERNEL/kernel-4.4
DEFCONFIG_OUT=${KERNEL_OUT}/generated

cat ./kernel/kernel-4.4/arch/arm64/configs/tegra18_android_defconfig \
    ./kernel/kernel-4.4/arch/arm64/configs/ext_config/ml-security-prod.config > \
    ${DEFCONFIG_OUT}/ml_combined_defconfig

time make -j${MAKETHREADS} -C ${T}/kernel/kernel-4.4/ -f Makefile ARCH=arm64 \
    LOCALVERSION="-tegra" CROSS_COMPILE=${CC}/aarch64-linux-android- \
    CROSS32CC=${CC32}/arm-eabi-gcc KCFLAGS=-mno-android O=${KERNEL_OUT} V=0 \
    KBUILD_RELSRC=../../../../../../../kernel/kernel-4.4 \
    DEFCONFIG_PATH=${DEFCONFIG_OUT} ml_combined_defconfig

set +x
echo ".config generated"
echo ".config generated"
echo ".config generated"
set -x

${T}/kernel/kernel-4.4/scripts/config --file ${KERNEL_OUT}/.config \
    --disable DEVMEM --disable DEVKMEM --disable DEBUG_KMEMLEAK \
    --disable FTRACE --disable DEBUG_FS

time make -j${MAKETHREADS} -C ${T}/kernel/kernel-4.4/ -f Makefile ARCH=arm64 \
    LOCALVERSION="-tegra" CROSS_COMPILE=${CC}/aarch64-linux-android- \
    CROSS32CC=${CC32}/arm-eabi-gcc KCFLAGS=-mno-android O=${KERNEL_OUT} V=0 \
    KBUILD_RELSRC=../../../../../../../kernel/kernel-4.4 \
    DEFCONFIG_PATH=${DEFCONFIG_OUT} oldconfig

time make -j${MAKETHREADS} -C ${T}/kernel/kernel-4.4/ -f Makefile ARCH=arm64 \
    LOCALVERSION="-tegra" CROSS_COMPILE=${CC}/aarch64-linux-android- \
    CROSS32CC=${CC32}/arm-eabi-gcc KCFLAGS=-mno-android O=${KERNEL_OUT} V=0 \
    KBUILD_RELSRC=../../../../../../../kernel/kernel-4.4 \
    NV_BUILD_KERNEL_DTS_ROOT=${T}/hardware/nvidia

set +x
echo "Done with kernel!!!"
echo "Done with kernel!!!"
echo "Done with kernel!!!"
set -x


