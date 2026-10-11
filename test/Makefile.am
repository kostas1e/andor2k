noinst_PROGRAMS = testClientSocket \
  testServerSocket \
  testCmdParser \
  testFitsFilename \
  testFitsHeaders \
  testFCC \
  testParsingFCCResponse \
  testNtpTime \
  testParallelAbort

if MOCK_CAMERA
AM_LDFLAGS = -L$(top_srcdir)/sim/lib
endif

MCXXFLAGS = \
	-std=c++17 \
	-g \
	-pg \
	-Wno-unused-variable \
	-Wall \
	-Wextra \
	-Werror \
	-pedantic \
	-W \
	-Wshadow \
	-Wdisabled-optimization \
	-DDEBUG

# testReadParameterFile_SOURCES   = test_read_parameter_file.cpp
# testReadParameterFile_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
# testReadParameterFile_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -lcfitsio -lm
# 
testClientSocket_SOURCES   = test_client_socket.cpp
testClientSocket_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
#testClientSocket_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -lcfitsio -lm
testClientSocket_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lpthread -lm

testServerSocket_SOURCES   = test_server_socket.cpp
testServerSocket_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
#testServerSocket_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -lcfitsio -lm
testServerSocket_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lpthread -lm

testCmdParser_SOURCES   = test_cmd_parser.cpp
testCmdParser_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
#testCmdParser_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -lcfitsio -lm
testCmdParser_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testFitsFilename_SOURCES   = test_fits_filename.cpp
testFitsFilename_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testFitsFilename_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm
#
#testFitsFilenameNext_SOURCES   = test_fits_filename.cpp
#testFitsFilenameNext_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
#testFitsFilenameNext_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -lcfitsio -lm

testFitsHeaders_SOURCES   = test_fits_header.cpp
testFitsHeaders_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testFitsHeaders_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testFCC_SOURCES   = test_fcc.cpp
testFCC_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testFCC_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testParsingFCCResponse_SOURCES   = test_parsing_fcc_response.cpp
testParsingFCCResponse_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testParsingFCCResponse_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testNtpTime_SOURCES   = test_ntp_time.cpp
testNtpTime_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testNtpTime_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm

testParallelAbort_SOURCES   = test_parallel_abort.cpp
testParallelAbort_CXXFLAGS  = $(MCXXFLAGS) -I$(top_srcdir)/src #-L$(top_srcdir)/src
testParallelAbort_LDADD     = $(top_srcdir)/src/libhelmosandor2k.la -landor -lcfitsio -lbz2 -lm -lpthread
