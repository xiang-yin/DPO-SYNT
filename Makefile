# project name
TARGET   = DES_Supervisor
SCATST   = ./test/scalability_test/scalability_test

# compiler flags
CXX      = g++
CXXFLAGS = -std=c++11

# file directories
SRCDIR   = src
OBJDIR   = obj
BINDIR   = bin
INCDIR   = include

# files
SOURCES  := $(wildcard $(SRCDIR)/*.cpp)
INCLUDES := $(wildcard $(INCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

all: release

# default make command - will compile program with $(CXXFLAGS)
#						 and the -O3 optomization
#						 Also turns off debugging
release: CXXFLAGS += -O3 -DNDEBUG
release: $(BINDIR)/$(TARGET)

scl: CXXFLAGS += -O3 -DNDEBUG
scl: $(SCATST)

# make debug - will compile program with $(CXXFLAGS) and the -g flag
#              also defines DEBUG so that "#ifdef DEBUG /*...*/ #endif" works
debug: CXXFLAGS += -g3 -DDEBUG -gdwarf-3
debug: clean $(BINDIR)/$(TARGET)

scldebug: CXXFLAGS += -g3 -DDEBUG -gdwarf-3
scldebug: sclclean $(SCATST)

# make profile - will compile "all" with $(CXXFLAGS) and the -pg flag
profile: CXXFLAGS += -pg -DNDEBUG
profile: clean $(BINDIR)/$(TARGET)

# compiles executable into the $(BINDIR) folder
$(BINDIR)/$(TARGET): $(OBJECTS)
	@$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS)
	@echo "Linking Complete!"

# individual object compliation
$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(CXX) $(CXXFLAGS) -c $< -o $@
	@echo "Compiled "$<" Successfully!"

# scalability test compilation
$(SCATST): $(SCATST).o $(OBJDIR)/FSM.o
	@$(CXX) $(CXXFLAGS) $(SCATST).o $(OBJDIR)/FSM.o -o $@
	@echo "Linking Complete!"

$(SCATST).o: $(SCATST).cpp 
	@$(CXX) $(CXXFLAGS) -c $< -o $@
	@echo "Compiled "$<" Successfully!"

# make clean - removes executable as well as all .o files in $(OBJDIR)
clean:
	@rm -f $(BINDIR)/$(TARGET) $(OBJECTS)
	@echo "Cleanup Complete!"

sclclean:
	@rm -f $(SCATST) $(SCATST).o