#/bin/sh

#arguments: 1 path, 2 file to wait for, 3 make options

cd $1
LASTFILE=`ls -Art | tail -n 1`
if mkdir Make.lock 2> /dev/null
then    # lock did not exist and was created successfully
    if [ "$LASTFILE" = $2 ]; then
	tput sgr0 && echo "         * UpToDate " $1/$2
    else
	tput setaf 3 && echo "       *** Make     " $1 && tput sgr0
	MAKEFLAGS='-j4 -k' make  -f Makefile $3
	if [ $? = 0 ] ; then # success
	    touch $2
	    tput setaf 2 && echo "       *** Done     " $1 && tput sgr0
	else # fail
	    tput bold && tput setaf 1
	    echo "     ***** FAILED   " $1/$2
	    tput sgr0
	fi
    fi
    rm -rf Make.lock
else
    tput setaf 0 && echo "         * Waiting  " $1/$2 && tput sgr0
    while [ -d Make.lock ]
    do
	sleep 0.2
    done
    tput setaf 0 && echo "         * DoneWait " $1/$2 && tput sgr0
fi

exit 0
