#
# Regular cron jobs for the openthread package
#
0 4	* * *	root	[ -x /usr/bin/openthread_maintenance ] && /usr/bin/openthread_maintenance
