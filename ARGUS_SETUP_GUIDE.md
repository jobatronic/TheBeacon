# Argus — Complete Setup Guide

This is everything, in order, for every box. Nothing here needs you to
remember anything from earlier conversations — it's all spelled out.

**The 4 boxes and their one job each:**

| Box | Job |
|---|---|
| Arduino Nano | Reads sensors (temp, door, water), controls relays |
| Orange Pi Zero | Talks to the Arduino, posts everything to MQTT, hosts the UPS, decides when it's time to shut everything down |
| Home Assistant (Pi 4) | Shows you the dashboard, listens for the "shut down" signal |
| Other computers (Klipper Pi, etc.) | Optional — also listen for the "shut down" signal |

Do them in this order: **Arduino → Orange Pi → Home Assistant → (optional) other computers.**


---

## Part 1 — The Arduino Nano

You've done this before. Nothing new here, just a reminder:

1. Pull the Nano out of its socket.
2. Flash it with `argus_controller.ino` using the Arduino IDE, exactly like last time.
3. Put it back in its socket, reconnect it to the Orange Pi's wires (the level-shifted connection you already tested).

That's it for the Arduino. It doesn't need internet, MQTT, or anything else — it just talks to whatever's on the other end of that wire.


---

## Part 2 — The Orange Pi Zero (the big one)

This box does the most work. We'll go slowly.

### 2.1 — Turn it on and log in

Power it up, SSH in from your work PC like you've done before.

### 2.2 — Install everything it needs, in one go

Copy-paste this whole line:

```bash
sudo apt update && sudo apt install -y python3-serial python3-paho-mqtt nut nut-client psmisc
```

What each piece is for, in plain words:
- `python3-serial` — lets our script talk to the Arduino
- `python3-paho-mqtt` — lets our script talk to the whiteboard (MQTT)
- `nut` + `nut-client` — the UPS-monitoring software
- `psmisc` — a tiny tool our shutdown script needs (`killall`)

### 2.3 — Copy 3 files onto the Orange Pi

From your work PC, copy these 3 files to the Orange Pi (scp, or however you've been doing it):

- `argus_bridge.py`
- `argus-bridge.service`
- `argus-ups-notify.sh`

Then, on the Orange Pi, put them where they belong:

```bash
sudo mkdir -p /opt/argus
sudo mv argus_bridge.py /opt/argus/
sudo mv argus-ups-notify.sh /opt/argus/
sudo chmod +x /opt/argus/argus-ups-notify.sh
```

(`argus-bridge.service` stays where you download it for now — it goes to a
different folder in step 2.6.)

### 2.4 — Fill in 4 blanks in `argus_bridge.py`

Open `/opt/argus/argus_bridge.py` and change these 4 lines near the top —
they currently say `CHANGE_ME` or a guessed value:

```python
SERIAL_PORT = "/dev/ttyS1"        # whatever `ls /dev/ttyS*` showed you
MQTT_BROKER_HOST = "CHANGE_ME"    # your Home Assistant Pi 4's IP address
MQTT_USERNAME = "CHANGE_ME"       # a login you create in Mosquitto (Part 3.1)
MQTT_PASSWORD = "CHANGE_ME"       # same
```

You'll create that MQTT username/password in Part 3 below — come back and
fill these in once you've done that.

### 2.5 — Quick test before making it permanent

Run it by hand and just watch:

```bash
python3 /opt/argus/argus_bridge.py
```

You should see it say it connected to the Arduino AND to MQTT. If either one
fails, the message tells you which — fix that before moving on. Press
Ctrl+C to stop it once you're happy.

### 2.6 — Make it run forever, automatically

```bash
sudo mv argus-bridge.service /etc/systemd/system/
```

Open that file and change `CHANGE_ME` to your actual Orange Pi username,
then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now argus-bridge
```

Now it starts on boot, keeps running after you log out, and restarts
itself if it ever crashes.

### 2.7 — Set up the UPS (5 small config files)

Plug the APC UPS's USB cable into the Orange Pi now, if you haven't.

Each file below already exists on your system with other stuff in it —
you're adding a few lines, not replacing the whole file. Use
`sudo nano <filename>` to edit each one.

**File: `/etc/nut/nut.conf`**
Find the `MODE=` line and set it to:
```
MODE=netserver
```
(This means "also answer questions from other computers on the network,"
not just this one.)

**File: `/etc/nut/ups.conf`**
Add this block at the bottom:
```
[apc]
    driver = usbhid-ups
    port = auto
    desc = "Server rack UPS"
```

**File: `/etc/nut/upsd.conf`**
Add this line:
```
LISTEN 0.0.0.0 3493
```
(Without this, only the Orange Pi itself could ask the UPS questions —
this opens it up to your other computers too.)

**File: `/etc/nut/upsd.users`**
Add these two blocks. These are two separate "logins" — one for Home
Assistant to just look at the numbers, one for the Orange Pi itself to be
allowed to pull the alarm:
```
[homeassistant]
    password = pick-a-password-here
    upsmon secondary

[monprimary]
    password = pick-a-different-password-here
    upsmon primary
```

**File: `/etc/nut/upsmon.conf`**
Find and change/add these lines (the file already has a template for most
of this — you're editing the existing lines, not starting from scratch):
```
MONITOR apc@localhost 1 monprimary pick-a-different-password-here primary

SHUTDOWNCMD "/sbin/shutdown -h now"
NOTIFYCMD /opt/argus/argus-ups-notify.sh
```
(Use the exact same password you picked for `monprimary` above in both
places.)

### 2.8 — Turn the UPS software on

```bash
sudo systemctl enable --now nut-server
sudo systemctl enable --now nut-monitor
sudo systemctl status nut-server
sudo systemctl status nut-monitor
```

Both should say `active (running)`, with no red error text.

### 2.9 — Check it's actually reading the UPS

```bash
upsc apc@localhost
```

You should see a wall of numbers: battery charge, status, load, etc. If
you see this, the UPS part is alive and working.

**One important thing to understand about `argus-ups-notify.sh`:**
This is the script that gives you the actual "2 minutes" countdown. Without
it, the UPS software would only shut things down based on the battery's
own built-in "I'm almost dead" signal — which isn't a fixed time, it
depends on how much stuff is plugged in. This script instead says: "the
moment the power goes out, start counting to 120 seconds, no matter what
the battery says." If you ever want a different number of minutes, that's
the one number (`WARN_SECONDS`) to change inside that file.


---

## Part 3 — Home Assistant (your Pi 4)

### 3.1 — Create MQTT login for the Orange Pi

Settings → Add-ons → Mosquitto broker → Configuration tab → add a login
(username + password) under `logins:`. Save, restart the add-on. Use this
same username/password back in Part 2.4 above.

### 3.2 — Confirm Argus shows up

Settings → Devices & Services → MQTT. You should see a device called
"Argus Rack Controller" with a bunch of sensors and 3 reset buttons — this
appears automatically, you don't build it by hand.

### 3.3 — Add the UPS integration

Settings → Devices & Services → Add Integration → search "Network UPS
Tools (NUT)". Host = Orange Pi's IP address, port `3493`, username =
`homeassistant`, password = whatever you picked for that login in Part
2.7.

### 3.4 — Find your UPS status entity's real name

Settings → Developer tools → States. In the search box type `status` and
look for one that belongs to your UPS (something like
`sensor.server_rack_ups_status`). Write down its exact name — you need it
in the next step.

### 3.5 — Add the "shut myself down" automation

Here's the important idea, explained simply: **the Orange Pi already did
the counting to 120 seconds.** Home Assistant doesn't need to count again
— it just needs to listen for the Orange Pi saying "okay, GO NOW" (that
signal is called `FSD` — Forced Shut Down) and shut down the instant it
hears it. If Home Assistant did its OWN 2-minute wait on top of that,
you'd end up waiting 4 minutes total, and out of sync with everything
else. So this automation reacts immediately, with no extra delay of its
own — the delay already happened on the Orange Pi.

Go to Settings → Automations & Scenes → Create Automation → skip the
visual editor → click the 3-dot menu → "Edit in YAML" → paste this in,
replacing the entity name with the one you found in step 3.4:

```yaml
alias: Argus UPS - Shutdown on Forced Shutdown Signal
description: >-
  Shuts down Home Assistant OS the instant the Orange Pi (acting as the
  UPS "primary") declares a forced shutdown. No extra delay here on
  purpose -- the Orange Pi already did the 2-minute countdown.
trigger:
  - platform: template
    value_template: >-
      {{ 'FSD' in (states('sensor.REPLACE_WITH_YOUR_ENTITY_NAME') | string).split() }}
condition: []
action:
  - service: hassio.host_shutdown
mode: single
```

Save it. That's the whole automation — 6 real lines once you strip the
comments.


---

## Part 4 — Other computers (optional)

Only do this for computers you actually want to auto-shutdown too, like
your Klipper Pi 3B+. Skip this section for anything you don't want
shutting itself off.

On that computer:

```bash
sudo apt install -y nut-client
```

Edit `/etc/nut/upsmon.conf` on THAT computer (not the Orange Pi):
```
MONITOR apc@<orange-pi-ip-address> 1 homeassistant <homeassistant-password> secondary
SHUTDOWNCMD "/sbin/shutdown -h now"
```

```bash
sudo systemctl enable --now nut-monitor
```

Done. No custom script needed here — it reacts to the same `FSD` signal
automatically, the same way Home Assistant does.


---

## Part 5 — Testing everything safely

Do these in order. Don't skip to the last one first.

1. **Arduino alone**: already done in earlier testing.
2. **Bridge script**: `python3 /opt/argus/argus_bridge.py` running, MQTT
   device visible in Home Assistant, one reset button tested from the
   dashboard.
3. **UPS reading**: `upsc apc@localhost` shows real numbers.
4. **The scary one — a real forced shutdown test.** This WILL actually
   shut down every machine that's listening, so only do it when you're
   ready for that:
   ```bash
   sudo upsmon -c fsd
   ```
   Everything configured to listen (the Orange Pi itself, Home Assistant
   if you added the automation, any other computer from Part 4) should
   shut down. This proves the whole chain works without needing to
   actually unplug anything.
5. **A real power-loss test** (once you trust everything above): unplug
   the UPS from the wall. You should see a `wall` message on the Orange
   Pi's screen/logs. Plug it back in within 2 minutes and confirm you see
   a "cancelled" message and nothing shuts down. Then, when you're
   genuinely ready, leave it unplugged the full 2 minutes and watch
   everything shut down together.


---

## If something's not working

| Symptom | Likely cause |
|---|---|
| Bridge script can't find the Arduino | Wrong `SERIAL_PORT` — recheck `ls /dev/ttyS*` |
| Bridge script can't reach MQTT | Wrong `MQTT_BROKER_HOST`, or the Mosquitto login doesn't match |
| No "Argus" device in Home Assistant | Bridge script isn't actually running, or MQTT integration isn't enabled in HA |
| `upsc apc@localhost` shows nothing | UPS not plugged into USB, or `driver = usbhid-ups` line wrong in `ups.conf` |
| Home Assistant doesn't shut down on `upsmon -c fsd` | Automation entity name in step 3.4 doesn't match your real sensor, or the automation is disabled |
| Other computer doesn't shut down | Its `MONITOR` line in `upsmon.conf` has the wrong IP, username, or password |
