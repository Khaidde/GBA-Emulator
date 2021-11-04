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
CFLAGS += -g
CFLAGS += -std=c++17 -Wall -Wextra -Werror -Wsign-conversion
endif

BIN_DIR ?= build/bin
OBJ_DIR ?= build/obj

OBJS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.o, $(shell ls src/*.cpp))
DEPS = $(patsubst src/%.cpp,$(OBJ_DIR)/%.d, $(shell ls src/*.cpp))
ifdef DEPS
-include ${DEPS}
endif

.PHONY: build clean

build: ${BIN_DIR}/gbaemu.exe

${BIN_DIR}/gbaemu.exe: ${OBJS}
	@${MKDIR} -p ${dir $@}
	${CC} ${CFLAGS} $^ -o $@

${OBJ_DIR}/%.o: src/%.cpp
	@${MKDIR} -p ${dir $@}
	${CC} ${CFLAGS} -c $< -MMD -MF $(@:.o=.d) -o $@

clean:
	${RM} -f ${BIN_DIR}/*
