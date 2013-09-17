CC=g++
CFLAGS= #-Wall 

#Cygwin use BAHHHH
LIBS = -lboost_system-mt
INC = -I/cygdrive/c/Git/vmmlib-master/vmmlib-master/include/

all: multirotor_attitude_control_h_infi.o
	$(CC) $(CFLAGS) udp_sim_link.cpp -o udp_sim_link $(LIBS) $(INC)

# main.o: main.cpp
# 	$(CC) $(CFLAGS) main.cpp

multirotor_attitude_control_h_infi.o: multirotor_attitude_control_h_infi.cpp
	$(CC) $(CFLAGS) $(INC) multirotor_attitude_control_h_infi.cpp

# hello.o: hello.cpp
# 	$(CC) $(CFLAGS) hello.cpp

# clean:
# 	rm -rf *o hello
