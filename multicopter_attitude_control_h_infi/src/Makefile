UNAME = Linux

ifeq ($(UNAME), Windows)
	LIBS = -lboost_system-mt
	INC = -I/cygdrive/c/Git/vmmlib-master/vmmlib-master/include/
endif
ifeq ($(UNAME), Linux)
	LIBS = -lboost_system -lpthread
	INC= -I/home/jacob/Packages/vmmlib-1.0/include/
endif

CXX = g++ -Wall -std=gnu++11
CC =

EXECUTABLE = udp_sim_link
OBJECTS = multirotor_attitude_control_h_infi.o udp_sim_link.o 

debug: CXX += -DDEBUG -g
debug: CC += -DDEBUG -g
debug: $(EXECUTABLE)

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^ $(LIBS) $(INC)

%.o: %.cpp
	$(CXX) -c $< $(INC)

clean:
	rm -f $(EXECUTABLE) $(OBJECTS)
