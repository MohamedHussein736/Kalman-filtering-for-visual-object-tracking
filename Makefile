

# Directories
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin
TARGET   = Tracker

LINKER   = g++
CC       = g++

SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(SRCDIR)/*.hpp)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
rm       = rm -f

LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_objdetect -lopencv_imgcodecs -lopencv_video
PATH_INCLUDES = /opt/installation/OpenCV-3.4.4/include
PATH_LIB = /opt/installation/OpenCV-3.4.4/lib

$(TARGET): $(OBJECTS)
	@$(LINKER) $(OBJECTS) -L$(PATH_LIB) $(LIBS) -o $@
	@echo "Linking complete"

$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(CC) $(CFLAGS) -c $< -I$(PATH_INCLUDES) -o $@
	@echo "Compiled "$<""

.PHONY: clean
clean:
	@$(rm) $(OBJECTS)
	@echo "Cleanup complete"

.PHONY: remove
remove: clean
	@$(rm) $(TARGET)
	@echo "Executable removed"

run: $(TARGET)
	 $(TARGET)
