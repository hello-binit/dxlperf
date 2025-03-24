# *** ENTER THE TARGET NAME HERE ***
TARGET      = cppsdk_feetech_reader_fps.out

# important directories used by assorted rules and other variables
DIR_DXL    = ./../SCServo_Linux
DIR_OBJS   = .objects

LD_LIBRARY_PATH := /home/hello-robot/repos/dxlperf/SCServo_Linux:$(LD_LIBRARY_PATH)
export LD_LIBRARY_PATH

# compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DXL)
LIBRARIES  += -l:libSCServo.a
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = cppsdk_feetech_reader_fps.cpp \
    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJECTS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***


#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo *.out

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ./%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@
