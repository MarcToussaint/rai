#/bin/sh

PATH=`pwd`
DATE=`/bin/date +"%y-%b-%d %H:%M:%S"`
cd $1
echo "         * Running " $1
echo "****" $1 $DATE >> $PATH/z.test-report
/usr/bin/timeout 20s ./x.exe -noInteractivity >> $PATH/z.test-report 2>&1
RETVAL=$?
if [ $RETVAL -eq 124 ]; then
    echo "     ***** TIMEOUT " $1
    echo "  ****TIMEOUT****" $1 >> $PATH/z.test-report
fi
if [ $RETVAL -eq 1 ]; then
    echo "     ***** FAILED  " $1
    echo "  ****FAILED****" $1 >> $PATH/z.test-report
fi
echo "         * Done " $1 >> $PATH/z.test-report
echo >> $PATH/z.test-report
