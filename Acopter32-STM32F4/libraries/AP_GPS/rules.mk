# Standard things
sp := $(sp).x
dirstack_$(sp) := $(d)
d := $(dir)
BUILDDIRS += $(BUILD_PATH)/$(d)

# Local flags
CFLAGS_$(d) := 

# Local rules and targets
cSRCS_$(d) :=

cppSRCS_$(d) :=
cppSRCS_$(d) += AP_GPS.cpp
cppSRCS_$(d) += AP_GPS_Glitch.cpp
cppSRCS_$(d) += AP_GPS_MTK.cpp
cppSRCS_$(d) += AP_GPS_MTK19.cpp
cppSRCS_$(d) += AP_GPS_NMEA.cpp
cppSRCS_$(d) += AP_GPS_SBP.cpp
cppSRCS_$(d) += AP_GPS_SIRF.cpp
cppSRCS_$(d) += AP_GPS_UBLOX.cpp
cppSRCS_$(d) += GPS_Backend.cpp

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