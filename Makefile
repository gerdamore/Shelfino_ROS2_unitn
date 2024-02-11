Sources=path_planning/src/RRT.cpp path_planning/src/Dubins.cpp  path_planning/src/Utils.cpp 
Target = dubins 

# general compiler settings
CXXFLAGS = -O3 -ffast-math 

all:
	$(CXX) $(CXXFLAGS) $(Sources) -o $(Target) $(LDFLAGS)

clean:
	@$(RM) $(Target)

.PHONY: clean