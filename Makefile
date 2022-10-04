
BASE = .

target: src

DEPEND = $(shell find rai -mindepth 1 -maxdepth 1 -not -name 'contrib' -printf "%f ")

################################################################################

src_paths =  $(shell find rai -mindepth 1 -maxdepth 1 -type d -not -name 'retired' -printf "%f ")

#contrib_paths =  $(shell find -L rai/contrib -mindepth 1 -maxdepth 1 -type d -not -name 'retired' -not -name '.git' -printf "%f ")

test_paths = $(shell find test -mindepth 3 -maxdepth 3 -name 'Makefile' -printf "%h ")

bin_paths = $(shell find bin -mindepth 2 -maxdepth 2 -name 'Makefile' -printf "%h ")

################################################################################

installUbuntuAll: force
	@echo "##### calling make installUbuntu in each lib"
	+@find rai -mindepth 1 -maxdepth 1 -type d -exec make installUbuntu -C {} \;

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

printDependAll: $(DEPEND:%=inPath_printDepend/%) printDepend

tests: $(test_paths:%=inPath_make/%) inPath_makeLib/ry

bin: $(bin_paths:%=inPath_make/%)

src: $(src_paths:%=inPath_makeLib/%)

ry: inPath_makeLib/ry

dependAll: $(src_paths:%=inPath_depend/%)

clean: $(src_paths:%=inPath_clean/%) cleanLocks

cleanStart: force
	@read -p " *** WARNING: This will rm ALL local files/changes (e.g. project/temporary/data files) - abort if you don't want to continue" yn
	git clean -f -d -x
	cp build/config.mk.default build/config.mk

paths: force
	@echo; echo ----------------------------------------
	@echo "  paths ";
	@echo ----------------------------------------; echo
	@echo "  src_paths =" "$(src_paths)"
	@echo "  contrib_paths =" "$(contrib_paths)"
	@echo "  test_paths =" "$(test_paths)"
	@echo "  bin_paths =" "$(bin_paths)"


################################################################################

INSTALL_PATH?=z.LOCAL

install: src bin
	mkdir -p $(INSTALL_PATH)/bin $(INSTALL_PATH)/lib/rai $(INSTALL_PATH)/include/rai
	cp bin/src_kinEdit/x.exe $(INSTALL_PATH)/bin/kinEdit
	cp lib/lib*.so $(INSTALL_PATH)/lib/rai
	@echo "copying headers into $(INSTALL_PATH)/include/rai"
	@eval $(shell cd rai; find . -maxdepth 1 -type d -printf "mkdir -p $(INSTALL_PATH)/include/rai/%f\; ")
	@eval $(shell cd rai; find . -maxdepth 2 -type f -name '*.h' -or -name '*.tpp' -printf "cp rai/%p $(INSTALL_PATH)/include/rai/%h/\; ")
	@find $(INSTALL_PATH)/include/rai
	@find $(INSTALL_PATH)/lib/rai


################################################################################


# test: setConfigFlag $(exa_paths:%=inPath_clean/%) cleanLocks $(exa_paths:%=inPath_make/%)

# setConfigFlag: force
# 	echo "RAI_TESTS = 1" > build/z.mk

runTests: tests
	@rm -f z.test-report
	@for p in $(test_paths); do build/run-path.sh $$p; done
	+@-make -C test/ry run clean


################################################################################

deletePotentiallyNonfreeCode: force
	@rm -Rf rai/Kin/SWIFT rai/Kin/SWIFT_decomposer rai/Geo/Lewiner rai/Geo/ply rai/Geo/GJK

################################################################################

include $(BASE)/build/generic.mk
