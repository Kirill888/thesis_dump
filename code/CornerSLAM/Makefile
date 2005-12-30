ODIR = o
CC   = gcc
CCC  = g++
INCLUDE_PATHS  = -I../Slam 
INCLUDE_PATHS += -I../xr4000
INCLUDE_PATHS += -I/home/users/kirill/DROS/include
CCFLAGS  = -Wall 
#CCFLAGS += -g
CCFLAGS += -O3 -march=pentium4 -mfpmath=sse -msse2
CCFLAGS += -DDO_TIME_LOG=0

LIBS = -lm -lXR4000 -lSLAM -lLineFitting  -lDROS -lGridMap    
LIB_PATHS  = -L../Slam
LIB_PATHS += -L../xr4000
LIB_PATHS += -L/home/users/kirill/DROS/lib
LDFLAGS =

BIN_DIR = ../bin

OBJ_SRCS = LaserGridMapper.cc LaserCornerExtractor.cc corner.cc hough.cc

APPS  = slam_app.cc corner_loc.cc extractCorners.cc


#Post Configure
APP_NAMES1  := ${APPS:%.cc=%}
APP_NAMES_ALL  := ${APP_NAMES1:%.c=%}

OBJS     := ${OBJ_SRCS:%.cc=$(ODIR)/%.o}
OBJS     := ${OBJS:%.c=$(ODIR)/%.o}

OBJS_ALL := ${OBJS} ${APPS:%.cc=$(ODIR)/%.o}
OBJS_ALL := ${OBJS_ALL:%.c=$(ODIR)/%.o}

ALL_SRCS := ${OBJ_SRCS} ${APPS}

ECHO_CHAR = @

#Dependency
DEPENDPROG = makedepend
DEPENDFILE = $(ODIR)/depend
DEPENDMSG = "\# Automatically generated dependencies $(shell date)"

all: $(APP_NAMES_ALL) Makefile ../Slam/libSLAM.a

$(APP_NAMES_ALL): % : $(ODIR)/%.o ${OBJS} ../Slam/libSLAM.a
	@echo Creating application $@
	$(ECHO_CHAR) $(CCC) $(LDFLAGS) $(LIB_PATHS) -o $@ $^ $(LIBS)

$(DEPENDFILE) : $(ALL_SRCS)
	@echo "$(MAKE): Generating include file dependencies";
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	$(ECHO_CHAR) rm -f $(ODIR)/depend
	$(ECHO_CHAR) $(DEPENDPROG) -p $(ODIR)/ -DDEPEND -s$(DEPENDMSG) \
                -- $(CPPFLAGS) $(CCFLAGS) $(INCLUDE_PATHS) -- -f- \
                $(ALL_SRCS) > $(DEPENDFILE)

include $(DEPENDFILE)

#Make Rules
$(ODIR)/%.o: %.cc Makefile ../Slam/libSLAM.a
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	@echo "Compiling C++ $<"
	$(ECHO_CHAR) $(CCC) $(CPPFLAGS) $(CCFLAGS) $(INCLUDE_PATHS) \
                -c -o $(ODIR)/$(<:%.cc=%.o) $<

$(ODIR)/%.o: %.c
	$(ECHO_CHAR) [ -d $(ODIR) ] || mkdir -p $(ODIR);
	@echo "Compiling C $<"
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

