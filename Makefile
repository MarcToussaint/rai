BASE = .

target: src
NAME   = $(shell basename $(PWD))

################################################################################

src_paths =  $(shell find rai -mindepth 1 -maxdepth 1 -type d -not -name 'extern' -not -name 'CMakeFiles' -not -name 'retired' -printf "%f ")

test_paths = $(shell find test -maxdepth 3 -name 'Makefile' -printf "%h ")

################################################################################

init: force
	@echo "##### calling make installUbuntuPackages in each lib"
	@find rai -mindepth 1 -maxdepth 1 -type d -exec make installUbuntuPackages -C {} \;

tests: $(test_paths:%=makePath/%)

src: $(src_paths:%=makeDepend/%)

depend: $(src_paths:%=dependPath/%)

clean: $(src_paths:%=cleanPath/%) cleanLocks

cleanStart: force
	@read -p " *** WARNING: This will rm ALL local files/changes (e.g. project/temporary/data files) - abort if you don't want to continue" yn
	git clean -f -d -x
	cp build/config.mk.default build/config.mk
	bin/createMakefileLinks.sh

################################################################################

# test: setConfigFlag $(exa_paths:%=cleanPath/%) cleanLocks $(exa_paths:%=makePath/%)

# setConfigFlag: force
# 	echo "MLR_TESTS = 1" > build/z.mk

runTests: tests
	@rm -f z.test-report
	@find test -mindepth 2 -maxdepth 2 -type d \
		-exec build/run-path.sh {} \;

################################################################################

doc:
	cd doc; doxygen MLR.doxy;
#	$(MAKE) -w -C doc guide doxy

reformatSources:
	astyle --options=rai/style.astyle "rai/MT/*.h" "rai/MT/*.cpp" "rai/MT/*.cxx"
	cd rai; find MT/ \( -name "*.h" -or -name "*.cpp" -or -name "*.cxx" \) -exec ./style.sed.sh {} \;

%.tgz: force
	bin/makeZip.sh $*

force:

################################################################################

include build/generic.mk
