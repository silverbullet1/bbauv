MAKEFLAGS = -r
BASEDIR ?= .
SHELL ?= /bin/sh
SUPPORT = $(BASEDIR)/support
E2CC = $(SUPPORT)/enum2cc
ENUM_CC = $(BASEDIR)/misc/enum2str.cc
ENUM_OBJ = $(BASEDIR)/misc/enum2str.o
ENUM_H = $(BASEDIR)/include/enum2str.h
CC = gcc
CXX = g++
CFLAGS = -g -Wall -O2
CXXFLAGS = $(CFLAGS)
LDLIBS = -lpthread
LIB_FOLDER = osx-x86_64

.PHONY: all
.SECONDARY:

container_SRC = example/Containers.cc
container_OBJ = example/Containers.o
container_EXE = example/Containers
servers_SRC = example/servers.cc
servers_OBJ = example/servers.o
servers_EXE = example/servers

EXE: $(container_EXE) $(servers_EXE)

all: $(EXE)

# identify host

HOST_OS = $(shell $(SUPPORT)/idenv -os)
HOST_ARCH = $(shell $(SUPPORT)/idenv -arch)

# identify target

ifndef $(TARGET)
	TARGET = $(HOST_OS)
	TARGET_ARCH = $(HOST_ARCH)
endif

ifeq ($(TARGET),linux)
	DEFS = -DLINUX
endif

ifeq ($(TARGET),osx)
	DEFS = -DOSX -D_OSX
endif

LIB_DSAAV = $(BASEDIR)/dsaav/$(LIB_FOLDER)/libdsaav-$(TARGET).a

CPPFLAGS = -I$(BASEDIR)/include\
            -I$(BASEDIR)/include/$(TARGET)\
            -I$(BASEDIR)/include/$(TARGET_ARCH)\
            $(DEFS)


#rules
$(container_OBJ):$(container_SRC) $(ENUM_H)
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<

$(container_EXE): $(container_OBJ) $(ENUM_OBJ) $(LIB_DSAAV)
	$(CXX) -o $@ $(LDFLAGS) $^ $(LDLIBS)
	
$(servers_OBJ):$(servers_SRC) $(ENUM_H)
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<

$(servers_EXE): $(servers_OBJ) $(ENUM_OBJ) $(LIB_DSAAV)
	$(CXX) -o $@ $(LDFLAGS) $^ $(LDLIBS)


# support for str2enum and enum2str

ENUM_SRC := $(shell cat enums.list)

$(ENUM_CC) $(ENUM_H): $(ENUM_SRC)
	$(E2CC) $(ENUM_CC) $(ENUM_H) $^

$(ENUM_OBJ):$(ENUM_CC) $(ENUM_H)
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -o $@ $<


clean::
	rm -f $(ENUM_OBJ) $(ENUM_CC) $(ENUM_H) $(container_OBJ) $(container_EXE) $(servers_OBJ) $(servers_EXE)