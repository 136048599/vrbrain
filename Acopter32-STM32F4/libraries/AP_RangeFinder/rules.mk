# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)

# Local flags
#CFLAGS_$(d) := -Wall -Werror
CFLAGS_$(d) := 

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) += AP_RangeFinder_analog.cpp
cppSRCS_$(d) += AP_RangeFinder_MaxsonarI2CXL.cpp
cppSRCS_$(d) += AP_RangeFinder_PulsedLightLRF.cpp
cppSRCS_$(d) += AP_RangeFinder_PX4.cpp
cppSRCS_$(d) += RangeFinder_Backend.cpp
cppSRCS_$(d) += RangeFinder.cpp


cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d) := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
             $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d) := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include $(DEPS_$(d))
d := $(dirstack_$(sp))
sp := $(basename $(sp))