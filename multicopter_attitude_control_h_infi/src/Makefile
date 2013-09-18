CC=g++
CFLAGS= -Wall -std=gnu++11

#Cygwin use BAHHHH
#LIBS = -lboost_system-mt
#Ubuntu use
LIBS = -lboost_system -lpthread
INC=

all: 
	$(CC) $(CFLAGS) udp_sim_link.cpp -o udp_sim_link $(LIBS) $(INC)

# main.o: main.cpp
# 	$(CC) $(CFLAGS) main.cpp

# factorial.o: factorial.cpp
# 	$(CC) $(CFLAGS) factorial.cpp

# hello.o: hello.cpp
# 	$(CC) $(CFLAGS) hello.cpp

# clean:
# 	rm -rf *o hello
