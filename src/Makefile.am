##
##  The library will be called: (lib)"helmosandor2k".
## ------------------------------------------------
##
lib_LTLIBRARIES = libhelmosandor2k.la

##
##  Only use the following flags for debuging purposes.
## ----------------------------------------------------
##
libhelmosandor2k_la_CXXFLAGS = \
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

##
##  Header files (distributed) installed in /$(includedir)/$(package name).
## ------------------------------------------------------------------------
##
pkginclude_HEADERS = \
        cpp_socket.hpp \
		andor_time_utils.hpp \
		andor2k.hpp \
		cppfits.hpp \
		andor2kd.hpp \
		fits_header.hpp \
        aristarchos.hpp \
	get_exposure.hpp \
	acquisition_reporter.hpp \
	acquisition_series_reporter.hpp \
	cbase64.hpp \
	IFilterWheel.hpp \
	DualWheelMapper.hpp \
	WheelConfig.hpp

if MOCK_WHEEL
pkginclude_HEADERS += MockFilterWheel.hpp
else
pkginclude_HEADERS += MaxonFilterWheel.hpp
endif

##
##  Source files (distributed).
## ----------------------------
##
dist_libhelmosandor2k_la_SOURCES = \
    andor2k.cpp \
	andor_time_utils.cpp \
	coarse_image_time.cpp \
	cpp_socket.cpp \
	cmd_parser.cpp \
	andor_tools.cpp \
	set_readout_mode.cpp \
	set_acquisition_mode.cpp \
	set_preampgain.cpp \
	cooling.cpp \
	fits_filenames.cpp \
	sys_shutdown.cpp \
	usage.cpp \
	get_acquisition.cpp \
	set_acquisition.cpp \
	resolve_image_parameters.cpp \
    vhsspeed.cpp \
	fits_header.cpp \
    aristarchos.cpp \
    socket_sprintf.cpp \
    cbase64.cpp \
    get_ntp_time.cpp \
	acquisition_reporter.cpp \
	acquisition_series_reporter.cpp \
    get_single_scan.cpp \
    get_rta_scan.cpp \
	abort_listener.cpp \
	save_as_fits.cpp \
	DualWheelMapper.cpp \
	WheelConfig.cpp

if MOCK_WHEEL
dist_libhelmosandor2k_la_SOURCES += MockFilterWheel.cpp
else
dist_libhelmosandor2k_la_SOURCES += MaxonFilterWheel.cpp
libhelmosandor2k_la_LIBADD = $(EPOS_LDFLAGS)
endif
