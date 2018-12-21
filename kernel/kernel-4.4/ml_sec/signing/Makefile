#
# Makefile for ML_SECURITY's signature verification
#

#place signing-common files here, these will be included in the build if *either* of the signing features below are set.
#(this entire Makefile only included if either of the signing features are set).
obj-y			:=


#ifdef example
#ifdef CONFIG_ML_SECURITY
#	obj-y		+=
#else
#	obj-y		+=
#endif

obj-$(CONFIG_MLSEC_ELF_SIGNING) += signature_verification.o

#build host utilities:
#---------------------
hostprogs-y   += extract_certificate_identifier
HOSTLOADLIBES_extract_certificate_identifier := -lcrypto
always        := $(hostprogs-y)

#signing development targets:
#---------------------
obj-$(CONFIG_MLSEC_ELF_SIGNING_DEVELOP) += dev/
