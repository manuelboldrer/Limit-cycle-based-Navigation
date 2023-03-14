
# compilation rule
# use make file.s

#
# result: dependencies
#         compilation rule

OPT  = -O3 -g0
CXX  = g++
WARN = -Wall
#INC  = -I/usr/local/include/eigen3

	
1: main.o 
	$(CXX) main.o -o run

# generic rule for compile C++-code
%.o: %.cc
	$(CXX) $(OPT) -c $<

%.o: %.cpp
	$(CXX) $(OPT) -c $<

clean:
	rm -f *.o *.s
	# $< = %.c
	# $@ = %.s
