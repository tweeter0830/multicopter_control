UNAME = Windows

ifeq ($(UNAME), Windows)
	LIBS = -lboost_system-mt
	INC = -I/cygdrive/c/Git/vmmlib-master/vmmlib-master/include/
endif
ifeq ($(UNAME), Linux)
	LIBS = -lboost_system -lpthread
	INC= -I/home/jacob/Packages/vmmlib-1.0/include/
endif

CC=g++
CFLAGS=-c -Wall -std=gnu++11

#Cygwin use BAHHHH
#Ubuntu use

all: multirotor_attitude_control_h_infi.o udp_sim_link.o
	$(CC) multirotor_attitude_control_h_infi.o udp_sim_link.o -o udp_sim_link $(LIBS) $(INC)

multirotor_attitude_control_h_infi.o: multirotor_attitude_control_h_infi.cpp
	$(CC) $(CFLAGS) $(INC) multirotor_attitude_control_h_infi.cpp

udp_sim_link.o: udp_sim_link.cpp
	$(CC) $(CFLAGS) $(INC) udp_sim_link.cpp
# clean:
# 	rm -rf *o hello
