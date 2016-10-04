#/bin/sh

cd $1
LASTFILE=`ls -Art | tail -n 1`
if [ "$LASTFILE" = '.lastMake' ]; then
    tput sgr0 && echo "         * UpToDate " $1
else
    if mkdir Make.lock 2> /dev/null
    then    # lock did not exist and was created successfully
	tput sgr0 && echo "       *** Make     " $1
	MAKEFLAGS='-j4 -k' make  -f makefile $2
	if [ $? -eq 0 ] ; then # success
	    date +'%y-%m-%d-%T' > .lastMake
	else # fail
	    tput bold; tput setaf 1
	    echo "     ***** FAILED   " $1
	    tput sgr0
	fi
	rm -rf Make.lock
    else
	tput sgr0 && echo "         * Waiting  " $1
	while [ -d $1/Make.lock ]
	do
	    sleep 0.2
	done
    fi
fi
