ODIR = o
CC   = gcc
CCC  = g++
AR   = ar
AR_FLAGS = -rcs
#INCLUDE_PATHS  = -I/home/users/kirill/KKK/include 
CCFLAGS  = -Wall 
#CCFLAGS += -g
CCFLAGS += -O3 -march=pentium4 -mfpmath=sse -msse2
#CCFLAGS += -DUSE_MEMSTAT
CCFLAGS += -DPFILT_DEBUG_LOG=0
CCFLAGS += -DDO_TIME_LOG=0
CCFLAGS += -DDUMP_LOOP_CLOSE_DATA=0
CCFLAGS += -DGRID_MAP_SAFETY_CHECK=0
CCFLAGS += -DSTORE_SENSOR_POSE=1
CCFLAGS += -DDUMP_TRANSIT=0


OBJ_SRCS = DataStore.cc MappingAgent.cc TimeTools.c geometry.cc global_vars.cc\
globalmap.cc graph.cc gridmap.cc image.cc kalman2.cc \
localmap.cc localiser.cc landmark2d.cc map.cc mapRegion.cc multiSLAM.cc\
odoCar.cc odoHolonomic.cc pfilter.cc random.cc simpleSLAM.cc simplex.cc\
slam.cc util.cc matlab.cc bisearch.cc LineLocaliser.cc LocaliserHype.cc\
MultiLocaliser.cc

ifdef USE_MEMSTAT
OBJ_SRC += memstat.cc
endif

LIB_NAME = SLAM
LIB_FILE = lib$(LIB_NAME).a

#Post Configure
OBJS                  := ${OBJ_SRCS:%.cc=$(ODIR)/%.o}
OBJS                  := ${OBJS:%.c=$(ODIR)/%.o}

OBJS_ALL := ${OBJS} ${APPS:%.cc=$(ODIR)/%.o}
OBJS_ALL := ${OBJS_ALL:%.c=$(ODIR)/%.o}

ALL_SRCS := ${OBJ_SRCS}

ECHO_CHAR = @

#Dependency
DEPENDPROG = makedepend
DEPENDFILE = $(ODIR)/depend
DEPENDMSG = "\# Automatically generated dependencies $(shell date)"

all: $(LIB_FILE)

$(DEPENDFILE) : $(ALL_SRCS)
	@echo "$(MAKE): Generating include file dependencies";
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	$(ECHO_CHAR) rm -f $(ODIR)/depend
	$(ECHO_CHAR) $(DEPENDPROG) -p $(ODIR)/ -DDEPEND -s$(DEPENDMSG) \
                -- $(CPPFLAGS) $(CCFLAGS) $(INCLUDE_PATHS) -- -f- \
                $(ALL_SRCS) > $(DEPENDFILE)

include $(DEPENDFILE)

%.a: $(OBJS_ALL)
	@echo "Building library file: $(LIB_FILE)"
	$(ECHO_CHAR) $(AR) $(AR_FLAGS) $(LIB_FILE) $(OBJS_ALL)

#Make Rules
$(ODIR)/%.o: %.cc Makefile
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	@echo "Compiling $<"
	$(ECHO_CHAR) $(CCC) $(CPPFLAGS) $(CCFLAGS) $(INCLUDE_PATHS) \
                -c -o $(ODIR)/$(<:%.cc=%.o) $<

$(ODIR)/%.o: %.c
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	@echo "Compiling $<"
	$(ECHO_CHAR) $(CC) $(CCFLAGS) $(INCLUDE_PATHS) \
                -c -o $(ODIR)/$(<:%.c=%.o) $<

clean: 
	@echo Cleaning up
	rm ${OBJS_ALL} $(DEPENDFILE)

#tgz: slam_backup.tgz
#slam_backup.tgz: ${ALL_SRCS}
#	@echo Making backup archive: $@
#	tar -cvzf slam_backup.tgz ${ALL_SRCS} *.h Makefile
#	cp -i slam_backup.tgz `date +bak/slam_%d%m%y.tgz`

