ODIR = o
CC   = gcc
CCC  = g++
AR   = ar
AR_FLAGS = -rcs
INCLUDE_PATHS  = -I../Slam
CCFLAGS  = -Wall 
#CCFLAGS += -g
CCFLAGS += -O3 -march=pentium4 -mfpmath=sse -msse2

OBJ_SRCS = DataSource.cc odoXR4000.cc


LIB_NAME = XR4000
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

