BUILD_TYPE ?= RELEASE

PLATFORM := $(shell uname -s)
ifneq ($(findstring MSYS,$(PLATFORM)),)
PLATFORM := windows32
endif

ifneq ($(PLATFORM),windows32)
$(warn Build not test on ${PLATFORM})
MAKE := make
MKDIR := mkdir
RM := rm
else
MAKE := $(shell which make)
MKDIR := $(shell which mkdir)
RM := $(shell which rm)
endif

ifeq ($(shell which clang++),)
$(error Could not find path to clang++)
else
CC := clang++
CFLAGS += -std=c++17 -Wall -Wextra -Werror -Wsign-conversion
LDFLAGS := -lshell32 -lSDL2main -lSDL2
endif

BIN_DIR ?= build/bin
OBJ_DIR ?= build/obj

SDL2_INCLUDE ?= C:/SDL2/include
SDL2_LIB ?= C:/SDL2/lib/x64

CORE_DIR := src/core

OBJS = $(patsubst ${CORE_DIR}/%.cpp,${OBJ_DIR}/%.o, $(shell ls ${CORE_DIR}/*.cpp))
DEPS = $(patsubst ${CORE_DIR}/%.cpp,${OBJ_DIR}/%.d, $(shell ls ${CORE_DIR}/*.cpp))
ifdef DEPS
-include ${DEPS}
endif

.PHONY: build build-debug clean

force-release:
	$(eval CFLAGS += -O3)

build: force-release ${BIN_DIR}/gbaemu.exe

force-debug:
	$(eval CFLAGS += -g -DDEBUG)

build-debug: force-debug ${BIN_DIR}/gbaemu.exe

${BIN_DIR}/gbaemu.exe: ${OBJS}
	@${MKDIR} -p ${dir $@}
	${CC} ${CFLAGS} -L${SDL2_LIB} $^ -o $@ ${LDFLAGS} -Wl,--subsystem,console
	cp ${SDL2_LIB}/SDL2.dll ${BIN_DIR}/SDL2.dll

${OBJ_DIR}/%.o: ${CORE_DIR}/%.cpp
	@${MKDIR} -p ${dir $@}
	${CC} ${CFLAGS} -I${SDL2_INCLUDE} -c $< -MMD -MF $(@:.o=.d) -o $@

clean:
	${RM} -f ${BIN_DIR}/* ${OBJ_DIR}/*
