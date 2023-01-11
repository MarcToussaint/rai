#/bin/sh

THISPATH=`pwd`
DATE=`/bin/date +"%y-%b-%d %H:%M:%S"`
cd $1
echo "         * Running " $1
echo "****" $1 $DATE >> $THISPATH/z.test-report
/usr/bin/timeout 60s ./x.exe -noInteractivity >> $THISPATH/z.test-report 2>&1
RETVAL=$?
if [ $RETVAL = 124 ]; then
    echo "     ***** TIMEOUT " $1
    echo "  ****TIMEOUT****" $1 $RETVAL >> $THISPATH/z.test-report
elif [ $RETVAL != 0 ]; then
    echo "     ***** FAILED  " $1 " with return code " $RETVAL " (2=NIY 3=NICO 134=HALT, 139=SegFault)"
    echo "  ****FAILED****" $1 $RETVAL >> $THISPATH/z.test-report
fi
echo "         * Done " $1 $RETVAL >> $THISPATH/z.test-report
echo >> $THISPATH/z.test-report
