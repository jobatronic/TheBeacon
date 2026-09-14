#!/bin/sh
#
# ═══════════════════════════════════════════════════════════════
#  ARGUS UPS NOTIFY SCRIPT — runs on the Orange Pi (upsmon primary)
# ═══════════════════════════════════════════════════════════════
#
#  upsmon calls this script on every UPS event, with NOTIFYTYPE
#  set to tell us which one.
#
#  upsmon's own built-in shutdown trigger is tied to the UPS
#  hardware's own low-battery signal, which depends on load and
#  isn't a fixed amount of time. This script instead gives a fixed,
#  predictable deadline: the moment power is lost (ONBATT), start
#  counting down WARN_SECONDS. If power comes back before that
#  (ONLINE), cancel it. If not, force a synchronized shutdown of
#  this box AND every secondary machine watching this same UPS,
#  via "upsmon -c fsd" (Forced Shut Down).
#
#  This is the standard, documented way to do a fixed-time UPS
#  shutdown with NUT — there's no simpler built-in equivalent.
#

WARN_SECONDS=120   # <-- "you have 2 minutes"

PATH=/sbin:/usr/sbin:/bin:/usr/bin

# If a previous countdown (still sleeping) gets sent SIGTERM
# because power came back, exit cleanly instead of continuing on
# to the shutdown at the bottom of the ONBATT case.
trap 'exit 0' TERM

case "$NOTIFYTYPE" in

    ONBATT)
        logger -t argus-upsmon "Power lost -- ${WARN_SECONDS}s to forced shutdown"
        wall "Power's out. Everything shuts down in ${WARN_SECONDS}s unless it returns." 2>/dev/null

        n=$WARN_SECONDS
        while [ "$n" -gt 0 ]; do
            sleep 1
            n=$((n - 1))
        done

        logger -t argus-upsmon "Countdown expired -- forcing shutdown (FSD)"
        wall "Time's up. Shutting down now." 2>/dev/null
        upsmon -c fsd
        ;;

    ONLINE)
        logger -t argus-upsmon "Power restored -- cancelling any pending countdown"
        wall "Power's back. Shutdown cancelled." 2>/dev/null

        # Kill any earlier copy of this script still in its sleep
        # loop from an ONBATT event that just got reversed. Needs
        # psmisc (sudo apt install -y psmisc) for killall.
        killall -s TERM "$(basename "$0")" 2>/dev/null
        ;;

esac

exit 0
