ifeq ($(strip $(PSL1GHT)),)
$(error "Please set PSL1GHT in your environment. export PSL1GHT=<path>")
endif

all:
	@$(MAKE) -C rbbroadphase --no-print-directory
	@$(MAKE) -C rbcollision --no-print-directory
	@$(MAKE) -C rbsolver --no-print-directory
	@$(MAKE) -C raycast --no-print-directory
	@$(MAKE) -C heightfluid --no-print-directory
	@$(MAKE) -C pclbroadphase --no-print-directory
	@$(MAKE) -C pclcollision --no-print-directory
	@$(MAKE) -C pclsolver --no-print-directory
	@$(MAKE) -C pclmesh --no-print-directory
	
clean:
	@$(MAKE) -C rbbroadphase clean --no-print-directory
	@$(MAKE) -C rbcollision clean --no-print-directory
	@$(MAKE) -C rbsolver clean --no-print-directory
	@$(MAKE) -C raycast clean --no-print-directory
	@$(MAKE) -C heightfluid clean --no-print-directory
	@$(MAKE) -C pclbroadphase clean --no-print-directory
	@$(MAKE) -C pclcollision clean --no-print-directory
	@$(MAKE) -C pclsolver clean --no-print-directory
	@$(MAKE) -C pclmesh clean --no-print-directory
	
.PHONY: all clean install
