bin_PROGRAMS = dandor2kd \
	testAndor

MCXXFLAGS = \
	-std=c++17 \
	-g \
	-pg \
	-Wall \
	-Wextra \
	-Werror \
	-pedantic \
	-W \
	-Wshadow \
	-Winline \
	-Wdisabled-optimization \
	-march=native \
	-DDEBUG

dandor2kd_SOURCES = andor2kd.cpp
dandor2kd_CXXFLAGS = $(MXXFLAGS) -I$(top_srcdir)/src
if MOCK_CAMERA
dandor2kd_LDADD = $(top_srcdir)/src/libhelmosandor2k.la -L$(top_srcdir)/sim/lib -landor -lcfitsio -lbz2 -lm
else
dandor2kd_LDADD = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm
endif

#dandor2k_client_SOURCES = andor2k_client.cpp
#dandor2k_client_CXXFLAGS = $(MXXFLAGS) -I$(top_srcdir)/src
#dandor2k_client_LDADD = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testAndor_SOURCES = test_andor.cpp
testAndor_CXXFLAGS = $(MXXFLAGS) -I$(top_srcdir)/src
if MOCK_CAMERA
testAndor_LDADD = $(top_srcdir)/src/libhelmosandor2k.la -L$(top_srcdir)/sim/lib -landor -lcfitsio -lbz2 -lm
else
testAndor_LDADD = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm
endif
