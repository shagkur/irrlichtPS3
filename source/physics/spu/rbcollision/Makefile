#---------------------------------------------------------------------------------
# Clear the implicit built in rules
#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------
ifeq ($(strip $(PSL1GHT)),)
$(error "Please set PSL1GHT in your environment. export PSL1GHT=<path>")
endif

include	$(PSL1GHT)/spu_rules

BUILD		:=	build

#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------
export BASEDIR		:= $(CURDIR)
export DEPSDIR		:= $(CURDIR)/$(BUILD)

export LD			:=	$(CXX)

#---------------------------------------------------------------------------------
else
#---------------------------------------------------------------------------------


#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------

TASKBIN	:= $(BASEDIR)/rbCollision

#---------------------------------------------------------------------------------
INCLUDES	:= -I$(BASEDIR) \
			   -I$(BASEDIR)/../../ \
			   $(LIBPSL1GHT_INC)

LIBPATHS	:= $(LIBPSL1GHT_LIB)

LIBS		:= -lspumars -lspudma -lspuatomic -lsputhread -lsimdmath

CFLAGS		:= -Os -Wall --param inline-unit-growth=10000 \
			   --param large-function-growth=10000 --param max-inline-insns-single=10000 \
			   $(MACHDEP) $(INCLUDES)
			   
CXXFLAGS	:= $(CFLAGS) -fno-exceptions
ASFLAGS		:= $(MACHDEP) -D__ASSEMBLY__ $(INCLUDES)

LDFLAGS		:=	$(MACHDEP) -Wl,-Map,$(notdir $(TASKBIN)).map

#---------------------------------------------------------------------------------
VPATH :=	$(BASEDIR) \
			$(BASEDIR)/../../rigidbody/common/

#---------------------------------------------------------------------------------
OBJS_TASK	:=	main.o \
				collobject.o contact.o boxboxdistance.o boxcapsuledistance.o boxspheredistance.o \
				capsulecapsuledistance.o capsulespheredistance.o \
				closestcontact.o closestcontactconvex.o closestcontactheightfield.o closestcontactlargemesh.o \
				heightfield.o heightfieldfunction.o contactheightfield.o \
				trianglesboxcontacts.o trianglesspherecontacts.o trianglesconvexcontacts.o trianglescapsulecontacts.o \
				convexboxdistance.o convexcapsuledistance.o convexconvexdistance.o convexspheredistance.o \
				contactcache.o gjksolver.o gjksupportfunc.o simplexsolver.o
			
all: $(BUILD)

#---------------------------------------------------------------------------------
$(BUILD):
#---------------------------------------------------------------------------------
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) task -C $(BUILD) -f $(CURDIR)/Makefile

#---------------------------------------------------------------------------------
$(TASKBIN).task: $(OBJS_TASK)

#---------------------------------------------------------------------------------
.PHONY: $(BUILD) task clean

#---------------------------------------------------------------------------------
task: $(TASKBIN).task

#---------------------------------------------------------------------------------
install: all
	@[ -d $(BASEDIR)/../../../../data ] || mkdir -p $(BASEDIR)/../../../../data
	@cp -fv $(TASKBIN).task $(BASEDIR)/../../../../data

#---------------------------------------------------------------------------------
clean:
	@echo clean ...
	@rm -rf $(BUILD) $(TASKBIN).task

-include $(DEPSDIR)/*.d
