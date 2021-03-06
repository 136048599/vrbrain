# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)
BUILDDIRS       += $(BUILD_PATH)/$(d)/boards/$(BOARD)

LIBRARY_INCLUDES += -I$(d)/comm -I$(d)/boards/$(BOARD)

# Local flags
#CFLAGS_$(d) := -Wall -Werror

# Local rules and targets
cSRCS_$(d)   :=  
cSRCS_$(d)   += boards/$(BOARD)/system_stm32f4xx.c

cppSRCS_$(d) := 
cppSRCS_$(d) += boards/$(BOARD)/$(BOARD).cpp
cppSRCS_$(d) += boards.cpp
cppSRCS_$(d) += ext_interrupts.cpp
cppSRCS_$(d) += HardwareTimer.cpp

sSRCS_$(d)   := 
sSRCS_$(d)   += boards/$(BOARD)/$(BOARD)_startup.S

cFILES_$(d)   := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)
sFILES_$(d)   := $(sSRCS_$(d):%=$(d)/%)

OBJS_$(d)     := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o)
OBJS_$(d)     += $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
OBJS_$(d)     += $(sFILES_$(d):%.S=$(BUILD_PATH)/%.o)

DEPS_$(d)     := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
