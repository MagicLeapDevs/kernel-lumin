#CC := gcc
#PLUGIN_SOURCE_FILES := pax_plugin.c
#PLUGIN_OBJECT_FILES := $(patsubst %.c,%.o,$(PLUGIN_SOURCE_FILES))
GCCPLUGINS_DIR := $(shell $(CC) -print-file-name=plugin)
#CFLAGS += -I$(GCCPLUGINS_DIR)/include -fPIC -O2 -Wall -W -std=gnu99

ifeq ($(PLUGINCC),$(HOSTCC))
HOSTLIBS := hostlibs
HOST_EXTRACFLAGS += -I$(GCCPLUGINS_DIR)/include -I$(src) -std=gnu99 -ggdb
export HOST_EXTRACFLAGS
else
HOSTLIBS := hostcxxlibs
HOST_EXTRACXXFLAGS += -I$(GCCPLUGINS_DIR)/include -I$(src) -std=gnu++98 -fno-rtti -fno-exceptions -fasynchronous-unwind-tables -ggdb -Wno-unused-parameter -Wno-narrowing -Wno-unused-variable
export HOST_EXTRACXXFLAGS
endif

export GCCPLUGINS_DIR HOSTLIBS

$(HOSTLIBS)-$(CONFIG_GRKERNSEC_RANDSTRUCT) += randomize_layout_plugin.so
$(HOSTLIBS)-$(CONFIG_GCC_PLUGIN_CONSTIFY) += constify_plugin.so

always := $($(HOSTLIBS)-y)

randomize_layout_plugin-objs := randomize_layout_plugin.o
constify_plugin-objs := constify_plugin.o

$(obj)/randomize_layout_plugin.o: $(objtree)/$(obj)/randomize_layout_seed.h

quiet_cmd_create_randomize_layout_seed = GENSEED  $@
	  cmd_create_randomize_layout_seed = \
		$(CONFIG_SHELL) $(srctree)/$(src)/gen-random-seed.sh $@ $(objtree)/include/generated/randomize_layout_hash.h $(BUILD_SEED)
$(objtree)/$(obj)/randomize_layout_seed.h: FORCE
		$(call if_changed,create_randomize_layout_seed)

targets += randomize_layout_seed.h randomize_layout_hash.h
