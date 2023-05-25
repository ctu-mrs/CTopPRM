OPSYS=$(shell uname)

PLATFORM=$(shell uname -p)
ARCH=.$(PLATFORM)

ifeq ($(OPSYS),FreeBSD)
   BOOST_CFLAGS=-I/usr/local/include
   BOOST_LDFLAGS=-L/usr/local/lib
else
   LOG4CXX_CPPFLAGS=$(shell pkg-config --cflags liblog4cxx)
   LOG4CXX_LDFLAGS=$(shell pkg-config --libs liblog4cxx)

   CAIRO_LDFLAGS:=-L/usr/X11/lib
   CAIRO_CFLAGS:=-I/usr/X11/include
endif

LOG4CXX_LDFLAGS+=-llog4cxx

CXX:=ccache $(CXX)

CXXFLAGS+=-std=c++17

CPPFLAGS+=$(LOCAL_CFLAGS)
LDFLAGS+=$(LOCAL_LDFLAGS)

tclap_CFLAGS=-I./lib/tclap-code/include

# RRT_LDFLAGS=-L./lib/rapid -lRAPID -L./lib/flann/install/lib/ -llz4
# RRT_CFLAGS=-I./lib/rapid -I./lib/ann/ann/include -I./lib/mpnn/MPNN/include -I./lib/flann/install/include  $(tclap_CFLAGS)

RRT_LDFLAGS=-L./lib/flann/install/lib/ -llz4
RRT_CFLAGS=-I./lib/ann/ann/include -I./lib/mpnn/MPNN/include -I./lib/flann/install/include  $(tclap_CFLAGS)

AGILICIOUS_LDFLAGS=-L./lib/agilicious/agilib/build -lagilib -pthread
AGILICIOUS_CFFLAGS=-I./lib/agilicious/agilib/include 

# MAV_TRAJECTORY_LDFLAGS=-L./lib/mav_trajectory_generation/build -lmav_trajectory_generation -lglog -lnlopt
# MAV_TRAJECTORY_CFFLAGS=-I./lib/mav_trajectory_generation/include -I/usr/include/eigen3 -I/usr/include/glog

CNPY_LDFLAGS=-L./lib/cnpy/install/lib -l:libcnpy.a -lz 
CNPY_CFFLAGS=-I./lib/cnpy/install/include 

CPPFLAGS+=-I./include $(LOG4CXX_CPPFLAGS) $(RRT_CFLAGS) $(CNPY_CFFLAGS) 
LDFLAGS+=$(LOG4CXX_LDFLAGS) $(RRT_LDFLAGS) $(CNPY_LDFLAGS) -lyaml-cpp 



#$(IMR-H_CFLAGS) 
#$(CAIRO_LDFLAGS) $(IMR-H-ALGORITHM) $(IMR-H-GUI_LDFLAGS) $(IMR-H_LDFLAGS)

#CXXFLAGS+= -g
CXXFLAGS+= -O3 -march=native
#-g -O3 
#-pg
#-march=native 
#CXXFLAGS+= 
#CXXFLAGS+=-std=c++11

