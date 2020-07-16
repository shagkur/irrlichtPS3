#---------------------------------------------------------------------------------
# Clear the implicit built in rules
#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------
ifeq ($(strip $(PSL1GHT)),)
$(error "Please set PSL1GHT in your environment. export PSL1GHT=<path>")
endif

include $(PSL1GHT)/ppu_rules

#---------------------------------------------------------------------------------
# TARGET is the name of the output
# BUILD is the directory where object files & intermediate files will be placed
# SOURCES is a list of directories containing source code
# INCLUDES is a list of directories containing extra header files
#---------------------------------------------------------------------------------
TARGET		:=	$(notdir $(CURDIR))
BUILD		:=	build

INCLUDES	:=	include \
				include/debug \
				source/physics
				
SOURCES		:=	source \
				source/debug \
				source/physics/rigidbody \
				source/physics/particle \
				source/physics/raycast \
				source/physics/heightfluid \
				source/physics/taskutil \
				source/physics/util

DATA		:=
SHADERS		:=	shaders
SPUTASK		:=	source/physics/spu/rbbroadphase source/physics/spu/rbcollision source/physics/spu/rbsolver \
				source/physics/spu/heightfluid source/physics/spu/raycast source/physics/spu/pclbroadphase \
				source/physics/spu/pclcollision source/physics/spu/pclsolver source/physics/spu/pclmesh

TITLE		:=	RSX Text - PSL1GHT
APPID		:=	RSXT00001
CONTENTID	:=	UP0001-$(APPID)_00-0000111122223333

#---------------------------------------------------------------------------------
# options for code generation
#---------------------------------------------------------------------------------

CFLAGS		=	-O2 -Wall -Wno-unused-variable -Wno-unused-but-set-variable -faligned-new \
				-mcpu=cell $(MACHDEP) $(INCLUDE)
CXXFLAGS	=	$(CFLAGS) -fno-exceptions

LDFLAGS		=	$(MACHDEP) -Wl,-Map,$(notdir $@).map

CGCFLAGS	=	$(CGINCLUDE)

#---------------------------------------------------------------------------------
# any extra libraries we wish to link with the project
#---------------------------------------------------------------------------------
LIBS	:=

#---------------------------------------------------------------------------------
# list of directories containing libraries, this must be the top level containing
# include and lib
#---------------------------------------------------------------------------------
LIBDIRS	:= $(PORTLIBS)

#---------------------------------------------------------------------------------
# no real need to edit anything past this point unless you need to add additional
# rules for different file extensions
#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------

export OUTPUT			:=	$(CURDIR)/$(TARGET)

export PHYSICSDIR		:=	$(CURDIR)/source/physics

export DEPSDIR			:=	$(CURDIR)/$(BUILD)

export BUILDDIR			:=	$(CURDIR)/$(BUILD)

export VPATH			:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) \
							$(foreach dir,$(DATA),$(CURDIR)/$(dir)) \
							$(foreach dir,$(SHADERS),$(CURDIR)/$(dir)) \
							$(foreach dir,$(SPUTASK),$(CURDIR)/$(dir)) \
							$(PHYSICSDIR)/rigidbody/common

#---------------------------------------------------------------------------------
# automatically build a list of object files for our project
#---------------------------------------------------------------------------------
CFILES			:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))
sFILES			:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.s)))
SFILES			:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.S)))
BINFILES		:=	$(foreach dir,$(DATA),$(notdir $(wildcard $(dir)/*.*)))
VCGFILES		:=	$(foreach dir,$(SHADERS),$(notdir $(wildcard $(dir)/*.vcg)))
FCGFILES		:=	$(foreach dir,$(SHADERS),$(notdir $(wildcard $(dir)/*.fcg)))
SPUTASKFILES	:=	$(foreach dir,$(SPUTASK),$(notdir $(wildcard $(dir)/*.task)))

EXTRACFILES		:=
EXTRACPPFILES	:=	collobject.cpp

VPOFILES		:=	$(VCGFILES:.vcg=.vpo)
FPOFILES		:=	$(FCGFILES:.fcg=.fpo)

#---------------------------------------------------------------------------------
# use CXX for linking C++ projects, CC for standard C
#---------------------------------------------------------------------------------
ifeq ($(strip $(CPPFILES)),)
	export LD	:=	$(CC)
else
	export LD	:=	$(CXX)
endif

export OFILES	:=	$(addsuffix .o,$(BINFILES)) \
					$(addsuffix .o,$(VPOFILES)) \
					$(addsuffix .o,$(FPOFILES)) \
					$(addsuffix .o,$(SPUTASKFILES)) \
					$(CPPFILES:.cpp=.o) $(CFILES:.c=.o) \
					$(sFILES:.s=.o) $(SFILES:.S=.o) \
					$(EXTRACPPFILES:.cpp=.o) $(EXTRACFILES:.c=.o)

#---------------------------------------------------------------------------------
# build a list of include paths
#---------------------------------------------------------------------------------
export INCLUDE		:=	$(foreach dir,$(INCLUDES), -I$(CURDIR)/$(dir)) \
						$(foreach dir,$(LIBDIRS),-I$(dir)/include) \
						$(LIBPSL1GHT_INC) \
						-I$(CURDIR)/$(BUILD)

export CGINCLUDE	:=	$(foreach dir,$(INCLUDES), -Wcg,-I$(CURDIR)/$(dir)) \
						$(foreach dir,$(LIBDIRS), -Wcg,-I$(dir)/include) \
						-Wcg,-I$(CURDIR)/$(BUILD)
						
#---------------------------------------------------------------------------------
# build a list of library paths
#---------------------------------------------------------------------------------
export LIBPATHS	:=	$(foreach dir,$(LIBDIRS),-L$(dir)/lib) \
					$(LIBPSL1GHT_LIB)

export OUTPUT	:=	$(CURDIR)/lib$(TARGET)

.PHONY: $(BUILD) clean

#---------------------------------------------------------------------------------
$(BUILD): physicstasks
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

#---------------------------------------------------------------------------------
all: $(BUILD)
	
#---------------------------------------------------------------------------------
physicstasks: 
	@$(MAKE) --no-print-directory -C $(PHYSICSDIR)/spu -f $(PHYSICSDIR)/spu/Makefile
	
#---------------------------------------------------------------------------------
clean:
	@echo clean ...
	@rm -fr $(BUILD) $(OUTPUT).a
	@$(MAKE) --no-print-directory -C $(PHYSICSDIR)/spu -f $(PHYSICSDIR)/spu/Makefile clean
#---------------------------------------------------------------------------------
install-headers:
	@[ -d $(PORTLIBS)/include/irrlicht ] || mkdir -p $(PORTLIBS)/include/irrlicht
	@[ -d $(PORTLIBS)/include/irrlicht/debug ] || mkdir -p $(PORTLIBS)/include/irrlicht/debug
	@cp -fv include/*.h $(PORTLIBS)/include/irrlicht
	@cp -fv include/debug/*.h $(PORTLIBS)/include/irrlicht/debug
	
#---------------------------------------------------------------------------------
install: all install-headers
	@[ -d $(PORTLIBS)/lib ] || mkdir -p $(PORTLIBS)/lib
	@cp -fv $(OUTPUT).a $(PORTLIBS)/lib

#---------------------------------------------------------------------------------
else

DEPENDS	:=	$(OFILES:.o=.d)

#---------------------------------------------------------------------------------
# main targets
#---------------------------------------------------------------------------------
$(OUTPUT).a: $(OFILES)

#---------------------------------------------------------------------------------
# This rule links in binary data with the .bin extension
#---------------------------------------------------------------------------------
%.bin.o	:	%.bin
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

#---------------------------------------------------------------------------------
%.task.o:	%.task
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

#---------------------------------------------------------------------------------
%.vpo.o	:	%.vpo
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

#---------------------------------------------------------------------------------
%.fpo.o	:	%.fpo
#---------------------------------------------------------------------------------
	@echo $(notdir $<)
	@$(bin2o)

-include $(DEPENDS)

#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------
